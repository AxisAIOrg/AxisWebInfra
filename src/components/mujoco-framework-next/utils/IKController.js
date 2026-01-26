import * as THREE from 'three';
import { getPosition, getQuaternion, toMujocoPos, toMujocoQuat } from '../mujocoUtils.js';

const DEFAULT_CONFIG = {
  endEffectorBodyName: null,
  jointNames: [],
  mode: 'auto', // 'auto' | 'free' | 'joint'
  translationGain: 1.0,
  rotationGain: 0.5,
  maxIterations: 5,
  stepLimit: 0.05,
  // IK cost weights (roughly matching the JAX example):
  // - Pose cost: pos_weight / ori_weight (acts like W on the 6D pose residual)
  // - Limit cost: penalize approaching joint limits
  posWeight: 50.0,
  oriWeight: 10.0,
  limitWeight: 100.0,

  // UX: when the user is only translating (XYZ), we usually want to strongly keep orientation
  // instead of allowing the solver to "spend" orientation error to satisfy position.
  // This reduces the "move xyz but EE rotates" feeling.
  holdOrientationOnTranslate: true,
  holdOriWeight: 200.0,
  // Hard lock: when translating, force targetQuaternion to stay fixed (copied from EE at start of translation).
  lockOrientationOnTranslate: true,

  // Trust-region / LM damping (lambda). Higher => more stable, but "softer".
  lambdaInitial: 1.0,
  lambdaFactor: 2.0,
  lambdaMin: 1e-5,
  lambdaMax: 1e10,
  stepQualityMin: 1e-3,
  lmMaxTrials: 6,

  // Joint-limit soft barrier region as a fraction of joint range.
  // Example: 0.1 means penalize when within 10% of either end of the allowed range.
  limitMarginFraction: 0.1,

  // Stall protection: if LM steps stop reducing error (often due to contact/force limits),
  // stop pushing to avoid drifting joints into limits.
  stallMinImprovement: 1e-4,
  stallMaxIterations: 2,

  // When stalled (typically contact/force-limit), snap target to current EE pose to prevent
  // target windup. Otherwise releasing contact causes an extra drift as IK "catches up".
  snapTargetToCurrentOnStall: true,

  // Dynamic-sim IK safety:
  // In running simulation, iterating multiple times per frame while only writing ctrl
  // can cause "integrator windup" (targets run away when contact blocks motion).
  runningIterations: 1,

  // Anti-windup: clamp desired ctrl target around current qpos.
  // This prevents joints from drifting to limits when the end-effector is blocked by contact.
  maxCtrlOffset: 0.25, // radians (or meters for slide joints)
};

export class IKController {
  constructor(mujoco, model, data, config = {}) {
    // console.log('[IKController] Creating IKController with config:', config);
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;
    this.config = { ...DEFAULT_CONFIG, ...config };

    this.textDecoder = new TextDecoder('utf-8');
    this.namesBuffer = new Uint8Array(model.names);

    this.endEffectorBodyId = this.resolveEndEffectorBodyId(this.config.endEffectorBodyName);
    this.mocapId = this.model.body_mocapid ? this.model.body_mocapid[this.endEffectorBodyId] : -1;
    // console.log('[IKController] End effector body ID:', this.endEffectorBodyId, 'Mocap ID:', this.mocapId);

    this.controlledDofs = this.resolveJointDofs(this.config.jointNames);
    // console.log('[IKController] Controlled DOFs:', this.controlledDofs);
    // Map qpos address -> joint id (for limit ranges)
    this.qposToJointIdMap = this.buildQposToJointIdMap();

    // Build mapping from joint qpos addresses to actuator indices
    this.qposToActuatorMap = this.buildQposToActuatorMap();
    // console.log('[IKController] Qpos to actuator map:', this.qposToActuatorMap);

    this.targetPosition = new THREE.Vector3();
    this.targetQuaternion = new THREE.Quaternion();
    this.currentPose = {
      position: new THREE.Vector3(),
      quaternion: new THREE.Quaternion(),
    };
    this.translationError = new THREE.Vector3();
    this.rotationError = new THREE.Vector3();
    this.axis = new THREE.Vector3();

    this.poseScratch = {
      position: new THREE.Vector3(),
      quaternion: new THREE.Quaternion(),
    };

    this.qposScratch = new Float64Array(this.data.qpos.length);
    this.targetInitialized = false;
    this.targetDirty = false;
    // If the last teleop intent was translation, hold orientation strongly.
    this._holdOrientation = true;
    this._lockedOrientationActive = false;
    this._lockedQuaternion = new THREE.Quaternion();

    this.lastSolveTime = 0;
    this._stallCount = 0;
    this._lastSolveError = null;
    this._lmLambda = this.config.lambdaInitial ?? 1.0;
    
    // CRITICAL: Sync actuator ctrl values to match current qpos values
    // This ensures the robot maintains its current pose when simulation starts
    this.syncCtrlFromQpos();
    
    // Initialize target position from current end effector position
    this.syncTargetsFromModel();
    // console.log('[IKController] IKController created successfully');
  }

  resolveEndEffectorBodyId(requestedName) {
    // console.log(`[IKController] Resolving end effector body: "${requestedName}"`);
    // console.log(`[IKController] Total bodies: ${this.model.nbody}`);
    
    // Print all body names for debugging
    const allBodyNames = [];
    for (let i = 0; i < this.model.nbody; i++) {
      const name = this.decodeName(this.model.name_bodyadr[i]);
      allBodyNames.push(name);
      if (i < 20) { // Print first 20 bodies
        // console.log(`[IKController] Body[${i}]: "${name}"`);
      }
    }
    // console.log(`[IKController] All body names (${allBodyNames.length}):`, allBodyNames);
    
    if (!requestedName) {
      const errorMsg = '[IKController] No end effector body name provided in config.';
      // console.error(errorMsg);
      throw new Error(errorMsg);
    }
    const bodyId = this.findIdByName(requestedName, this.model.name_bodyadr, this.model.nbody);
    if (bodyId === -1) {
      const available = allBodyNames.filter(n => n.toLowerCase().includes('hand'));
      // console.error(`[IKController] Unable to locate required body "${requestedName}". Available bodies containing "hand":`, available);
      throw new Error(`[IKController] Required end effector body "${requestedName}" not found in model.`);
    }
    // console.log(`[IKController] Found body "${requestedName}" at ID: ${bodyId}`);
    return bodyId;
  }

  resolveJointDofs(jointNames = []) {
    // console.log(`[IKController] Resolving joint DOFs for:`, jointNames);
    // console.log(`[IKController] Total joints: ${this.model.njnt}`);
    
    // Print all joint names for debugging
    const allJointNames = [];
    for (let i = 0; i < this.model.njnt; i++) {
      const name = this.decodeName(this.model.name_jntadr[i]);
      allJointNames.push(name);
      if (i < 20) { // Print first 20 joints
        // console.log(`[IKController] Joint[${i}]: "${name}"`);
      }
    }
    // console.log(`[IKController] All joint names (${allJointNames.length}):`, allJointNames);
    
    if (!jointNames.length) {
      // console.log(`[IKController] No joint names provided, returning empty DOFs`);
      return [];
    }
    const hinges = [];
    for (const name of jointNames) {
      const jointId = this.findIdByName(name, this.model.name_jntadr, this.model.njnt);
      if (jointId === -1) {
        // console.warn(`[IKController] Joint "${name}" not found; skipping.`);
        // console.warn(`[IKController] Available joints containing "${name}":`, 
        //   allJointNames.filter(n => n.toLowerCase().includes(name.toLowerCase())));
        continue;
      }
      const type = this.model.jnt_type[jointId];
      const supported = (type === this.mujoco.mjtJoint.mjJNT_HINGE.value ||
                         type === this.mujoco.mjtJoint.mjJNT_SLIDE.value);
      if (!supported) {
        // console.warn(`[IKController] Joint "${name}" is not hinge/slide; currently unsupported in numeric IK.`);
        continue;
      }
      const addr = this.model.jnt_qposadr[jointId];
      // console.log(`[IKController] Found joint "${name}" at ID: ${jointId}, qpos address: ${addr}`);
      hinges.push(addr);
    }
    // console.log(`[IKController] Resolved ${hinges.length} joint DOFs:`, hinges);
    return hinges;
  }

  buildQposToJointIdMap() {
    const map = new Map();
    for (let j = 0; j < this.model.njnt; j++) {
      const addr = this.model.jnt_qposadr[j];
      if (addr >= 0) {
        map.set(addr, j);
      }
    }
    return map;
  }

  buildQposToActuatorMap() {
    // Map from qpos address to actuator index
    // Based on XML: actuator name matches joint name exactly (e.g., "franka/panda_joint1")
    const map = new Map();
    
    // In MuJoCo, mjTRN_JOINT = 0 (joint transmission)
    // Other types: mjTRN_SLIDERCRANK=1, mjTRN_TENDON=2, mjTRN_SITE=3, mjTRN_BODY=4
    const mjTRN_JOINT = 0;
    
    // console.log('[IKController] ========== Building Actuator Map ==========');
    // console.log(`[IKController] Total actuators (model.nu): ${this.model.nu}`);
    // console.log(`[IKController] Total joints (model.njnt): ${this.model.njnt}`);
    // console.log(`[IKController] ctrl array length (data.ctrl.length): ${this.data.ctrl.length}`);
    
    // Build joint name -> qposAddr map
    const jointNameToQposAddr = new Map();
    const jointNameToJointId = new Map();
    for (let j = 0; j < this.model.njnt; j++) {
      const jointName = this.decodeName(this.model.name_jntadr[j]);
      const qposAddr = this.model.jnt_qposadr[j];
      if (qposAddr >= 0) {
        jointNameToQposAddr.set(jointName, qposAddr);
        jointNameToJointId.set(jointName, j);
      }
    }
    
    // Print all actuators with their target joints
    // console.log('[IKController] All actuators:');
    for (let a = 0; a < this.model.nu; a++) {
      const actuatorName = this.decodeName(this.model.name_actuatoradr[a]);
      const trnType = this.model.actuator_trntype ? this.model.actuator_trntype[a] : -1;
      const trnId = this.model.actuator_trnid ? this.model.actuator_trnid[a] : -1;
      const targetJointName = trnId >= 0 && trnId < this.model.njnt ? 
        this.decodeName(this.model.name_jntadr[trnId]) : 'N/A';
      // console.log(`  actuator[${a}]: "${actuatorName}", trnType=${trnType}, trnId=${trnId}, targetJoint="${targetJointName}", ctrl[${a}]=${this.data.ctrl[a]}`);
    }
    
    // Print all joints
    // console.log('[IKController] All joints:');
    for (let j = 0; j < this.model.njnt; j++) {
      const jointName = this.decodeName(this.model.name_jntadr[j]);
      const qposAddr = this.model.jnt_qposadr[j];
      const jointType = this.model.jnt_type[j];
      // console.log(`  joint[${j}]: "${jointName}", qposAddr=${qposAddr}, type=${jointType}`);
    }
    
    // Build actuator name -> index map for quick lookup
    const actuatorNameToIndex = new Map();
    for (let a = 0; a < this.model.nu; a++) {
      const actuatorName = this.decodeName(this.model.name_actuatoradr[a]);
      actuatorNameToIndex.set(actuatorName, a);
    }

    const findActuatorIdx = (jointName) => {
      // Exact match
      if (actuatorNameToIndex.has(jointName)) { return actuatorNameToIndex.get(jointName); }
      // Prefixed path match (e.g., "franka/panda_joint1")
      if (actuatorNameToIndex.has(`franka/${jointName}`)) { return actuatorNameToIndex.get(`franka/${jointName}`); }
      // Suffix match
      for (const [actuatorName, idx] of actuatorNameToIndex.entries()) {
        if (actuatorName.endsWith(`/${jointName}`) || actuatorName.endsWith(jointName)) {
          return idx;
        }
      }
      return undefined;
    };

    // console.log('[IKController] Building map by joint order from config...');
    for (const jointName of this.config.jointNames) {
      const qposAddr = jointNameToQposAddr.get(jointName);
      if (qposAddr === undefined) {
        // console.error(`[IKController]   Joint "${jointName}" not found in model (jointNameToQposAddr missing)`);
        continue;
      }

      const actuatorIdx = findActuatorIdx(jointName);
      if (actuatorIdx === undefined) {
        // console.error(`[IKController]   No actuator found for joint "${jointName}"`);
        continue;
      }

      if (map.has(qposAddr)) {
        const existingActuatorIdx = map.get(qposAddr);
        const existingActuatorName = this.decodeName(this.model.name_actuatoradr[existingActuatorIdx]);
        // console.warn(`[IKController] WARNING: qpos[${qposAddr}] (joint "${jointName}") already mapped to actuator[${existingActuatorIdx}] "${existingActuatorName}"`);
        // console.warn(`[IKController]   Overriding with actuator[${actuatorIdx}] "${this.decodeName(this.model.name_actuatoradr[actuatorIdx])}"`);
      }

      map.set(qposAddr, actuatorIdx);
      // console.log(`[IKController] ✓ Ordered map: joint "${jointName}" qpos[${qposAddr}] -> actuator[${actuatorIdx}] "${this.decodeName(this.model.name_actuatoradr[actuatorIdx])}"`);
    }
    
    // console.log(`[IKController] Built qpos-to-actuator map with ${map.size} entries`);
    
    // Print controlled DOFs and their mappings - VERIFY CORRECTNESS
    // console.log('[IKController] Controlled DOFs and their actuator mappings:');
    for (let i = 0; i < this.controlledDofs.length; i++) {
      const qposAddr = this.controlledDofs[i];
      const actuatorIdx = map.get(qposAddr);
      let jointName = 'unknown';
      let jointId = -1;
      for (let j = 0; j < this.model.njnt; j++) {
        if (this.model.jnt_qposadr[j] === qposAddr) {
          jointName = this.decodeName(this.model.name_jntadr[j]);
          jointId = j;
          break;
        }
      }
      if (actuatorIdx !== undefined) {
        const actuatorName = this.decodeName(this.model.name_actuatoradr[actuatorIdx]);
        // Verify: check if actuator actually controls this joint
        const actuatorTrnId = this.model.actuator_trnid[actuatorIdx];
        const actuatorTrnType = this.model.actuator_trntype[actuatorIdx];
        const isCorrect = (actuatorTrnType === mjTRN_JOINT && actuatorTrnId === jointId);
        const status = isCorrect ? '✓' : '✗ WRONG!';
        // console.log(`  DOF[${i}]: qpos[${qposAddr}] (joint[${jointId}] "${jointName}") -> actuator[${actuatorIdx}] "${actuatorName}" ${status}`);
        if (!isCorrect) {
          // console.error(`    Expected actuator to control joint[${jointId}], but actuator[${actuatorIdx}] controls joint[${actuatorTrnId}]`);
        }
      } else {
        // console.error(`  DOF[${i}]: qpos[${qposAddr}] (joint "${jointName}") -> NO ACTUATOR FOUND`);
      }
    }
    
    // Verify all controlled DOFs have actuators
    const missingActuators = [];
    for (let i = 0; i < this.controlledDofs.length; i++) {
      const qposAddr = this.controlledDofs[i];
      if (!map.has(qposAddr)) {
        // Find joint name for this qpos address
        let jointName = 'unknown';
        for (let j = 0; j < this.model.njnt; j++) {
          if (this.model.jnt_qposadr[j] === qposAddr) {
            jointName = this.decodeName(this.model.name_jntadr[j]);
            break;
          }
        }
        missingActuators.push({ qposAddr, jointName });
      }
    }
    
    if (missingActuators.length > 0) {
      // console.error('[IKController] Missing actuators for controlled joints:', missingActuators);
      // console.error('[IKController] Controlled DOFs:', this.controlledDofs);
      // console.error('[IKController] Available actuators:');
      for (let a = 0; a < this.model.nu; a++) {
        const actuatorName = this.decodeName(this.model.name_actuatoradr[a]);
        const trnType = this.model.actuator_trntype[a];
        const trnId = this.model.actuator_trnid[a];
        let jointName = 'N/A';
        if (trnType === mjTRN_JOINT && trnId >= 0 && trnId < this.model.njnt) {
          jointName = this.decodeName(this.model.name_jntadr[trnId]);
        }
        // console.error(`  actuator[${a}]: "${actuatorName}", trnType=${trnType}, trnId=${trnId}, joint="${jointName}"`);
      }
      // console.error('[IKController] All joints:');
      for (let j = 0; j < this.model.njnt; j++) {
        const jointName = this.decodeName(this.model.name_jntadr[j]);
        const qposAddr = this.model.jnt_qposadr[j];
        // console.error(`  joint[${j}]: "${jointName}", qposAddr=${qposAddr}`);
      }
      const errorMsg = `[IKController] Missing actuators for ${missingActuators.length} controlled joints: ${missingActuators.map(m => `${m.jointName}(qpos[${m.qposAddr}])`).join(', ')}. Cannot proceed without actuators.`;
      throw new Error(errorMsg);
    }
    
    // console.log('[IKController] ========== Actuator Map Complete ==========');
    return map;
  }

  findIdByName(targetName, addressArray, count) {
    if (!targetName || !addressArray) { return -1; }
    for (let i = 0; i < count; i++) {
      const start = addressArray[i];
      const name = this.decodeName(start);
      if (name === targetName) { return i; }
    }
    return -1;
  }

  decodeName(start) {
    let end = start;
    while (end < this.namesBuffer.length && this.namesBuffer[end] !== 0) { end++; }
    return this.textDecoder.decode(this.namesBuffer.subarray(start, end));
  }

  syncCtrlFromQpos() {
    // Sync all actuator ctrl values to match current qpos values
    // This ensures the robot maintains its current pose when simulation starts
    // console.log('[IKController] Syncing ctrl values from qpos...');
    for (let i = 0; i < this.controlledDofs.length; i++) {
      const qposAddr = this.controlledDofs[i];
      const actuatorIdx = this.qposToActuatorMap.get(qposAddr);
      if (actuatorIdx !== undefined && actuatorIdx >= 0 && actuatorIdx < this.data.ctrl.length) {
        const currentQpos = this.data.qpos[qposAddr];
        this.data.ctrl[actuatorIdx] = currentQpos;
        // Find joint name for logging
        let jointName = 'unknown';
        for (let j = 0; j < this.model.njnt; j++) {
          if (this.model.jnt_qposadr[j] === qposAddr) {
            jointName = this.decodeName(this.model.name_jntadr[j]);
            break;
          }
        }
        const actuatorName = this.decodeName(this.model.name_actuatoradr[actuatorIdx]);
        if (i < 7) { // Only print first 7 to avoid spam
          // console.log(`[IKController]   Synced: joint "${jointName}" qpos[${qposAddr}]=${currentQpos.toFixed(4)} -> actuator[${actuatorIdx}] "${actuatorName}" ctrl[${actuatorIdx}]=${currentQpos.toFixed(4)}`);
        }
      } else {
        let jointName = 'unknown';
        for (let j = 0; j < this.model.njnt; j++) {
          if (this.model.jnt_qposadr[j] === qposAddr) {
            jointName = this.decodeName(this.model.name_jntadr[j]);
            break;
          }
        }
        // console.warn(`[IKController]   WARNING: No actuator found for joint "${jointName}" qpos[${qposAddr}]`);
      }
    }
    // console.log('[IKController] Initial ctrl array:', 
    //   Array.from(this.data.ctrl).map((v, i) => `ctrl[${i}]=${v.toFixed(4)}`).join(', '));
  }

  syncTargetsFromModel() {
    getPosition(this.data.xpos, this.endEffectorBodyId, this.targetPosition);
    // Read quaternion WITHOUT swizzle first to check raw values
    getQuaternion(this.data.xquat, this.endEffectorBodyId, this.targetQuaternion, true);
    this.targetInitialized = true;
    this.targetDirty = false;
    
    // Convert quaternion to Euler angles for better understanding
    const euler = new THREE.Euler().setFromQuaternion(this.targetQuaternion, 'XYZ');
    const eulerDeg = {
      x: THREE.MathUtils.radToDeg(euler.x),
      y: THREE.MathUtils.radToDeg(euler.y),
      z: THREE.MathUtils.radToDeg(euler.z)
    };
    
    // Check if quaternion is identity (1, 0, 0, 0)
    const isIdentity = Math.abs(this.targetQuaternion.w - 1.0) < 1e-6 &&
                       Math.abs(this.targetQuaternion.x) < 1e-6 &&
                       Math.abs(this.targetQuaternion.y) < 1e-6 &&
                       Math.abs(this.targetQuaternion.z) < 1e-6;
    
    // Also read raw MuJoCo quaternion values (before swizzle)
    const rawQuatAddr = this.endEffectorBodyId * 4;
    const rawQuat = {
      w: this.data.xquat[rawQuatAddr + 0],
      x: this.data.xquat[rawQuatAddr + 1],
      y: this.data.xquat[rawQuatAddr + 2],
      z: this.data.xquat[rawQuatAddr + 3]
    };
    
    // console.log('[IKController] ========== Synced target from model ==========');
    // console.log('[IKController] End Effector Position:', {
    //   x: this.targetPosition.x.toFixed(4),
    //   y: this.targetPosition.y.toFixed(4),
    //   z: this.targetPosition.z.toFixed(4)
    // });
    // console.log('[IKController] End Effector Rotation (Quaternion):', {
    //   w: this.targetQuaternion.w.toFixed(6),
    //   x: this.targetQuaternion.x.toFixed(6),
    //   y: this.targetQuaternion.y.toFixed(6),
    //   z: this.targetQuaternion.z.toFixed(6),
    //   isIdentity: isIdentity ? 'YES (1,0,0,0)' : 'NO'
    // });
    // console.log('[IKController] End Effector Rotation (Euler angles, degrees):', {
    //   roll_X: eulerDeg.x.toFixed(2),
    //   pitch_Y: eulerDeg.y.toFixed(2),
    //   yaw_Z: eulerDeg.z.toFixed(2)
    // });
    // console.log('[IKController] Raw MuJoCo Quaternion (before swizzle):', {
    //   w: rawQuat.w.toFixed(6),
    //   x: rawQuat.x.toFixed(6),
    //   y: rawQuat.y.toFixed(6),
    //   z: rawQuat.z.toFixed(6)
    // });
    // console.log('[IKController] ==============================================');
  }

  /**
   * Reset IK targets and orientation locks to the current end-effector pose.
   * Useful when the task/objects are reset so we don't keep a stale target orientation.
   */
  resetTargetsFromModel() {
    // Clear locks / mode flags
    this._lockedOrientationActive = false;
    this._holdOrientation = true;
    this._lockedQuaternion.identity();

    // Re-sync targets from current model state
    this.targetInitialized = false;
    this.targetDirty = false;
    this.syncTargetsFromModel();
  }

  adjustTargetPosition(delta) {
    if (!this._adjustPosCallCount) this._adjustPosCallCount = 0;
    this._adjustPosCallCount++;
    if (this._adjustPosCallCount <= 3) {
      // console.log('[IKController] adjustTargetPosition called:', {
      //   delta: delta,
      //   targetInitialized: this.targetInitialized
      // });
    }
    if (!this.targetInitialized) { 
      this.syncTargetsFromModel(); 
    }

    if (this.config.lockOrientationOnTranslate) {
      this.mujoco.mj_forward(this.model, this.data);
      this.readCurrentPose();
      if (!this._lockedOrientationActive) {
        this._lockedQuaternion.copy(this.currentPose.quaternion).normalize();
        this._lockedOrientationActive = true;
      }
      this.targetQuaternion.copy(this._lockedQuaternion).normalize();
    }

    this.targetPosition.add(delta);
    this._holdOrientation = true;
    this.targetDirty = true;
  }

  adjustTargetOrientation(deltaEuler) {
    if (!this._adjustRotCallCount) this._adjustRotCallCount = 0;
    this._adjustRotCallCount++;
    if (this._adjustRotCallCount <= 3) {
      // console.log('[IKController] adjustTargetOrientation called:', {
      //   deltaEuler: deltaEuler,
      //   targetInitialized: this.targetInitialized
      // });
    }
    if (!this.targetInitialized) { 
      this.syncTargetsFromModel(); 
    }
    const deltaQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(
      deltaEuler.x, deltaEuler.y, deltaEuler.z, 'XYZ'));
    this.targetQuaternion.multiply(deltaQuat).normalize();
    this._holdOrientation = false;
    this._lockedOrientationActive = false;
    this.targetDirty = true;
  }

  update(timestampMS, isPaused = false) {
    if (!this.targetInitialized) { 
      // console.log('[IKController] Initializing target from model...');
      this.syncTargetsFromModel(); 
    }

    const useMocap = (this.config.mode === 'free' || this.config.mode === 'auto') && this.mocapId >= 0;
    if (useMocap) {
      if (this.targetDirty) {
        // console.log('[IKController] Using mocap mode, mocapId:', this.mocapId);
      }
      this.applyMocapTarget();
      return;
    }

    if (!this.controlledDofs.length) {
      // Only warn once per second to avoid spam
      if (!this._lastNoDofWarning || (timestampMS - this._lastNoDofWarning) > 1000) {
        // console.warn('[IKController] No controlled DOFs available, cannot solve IK');
        this._lastNoDofWarning = timestampMS;
      }
      return;
    }

    if (!this.targetDirty) { return; }
    
    this.lastSolveTime = timestampMS;

    // In dynamic simulation (not paused), the robot does not "teleport" to our ctrl targets.
    // Doing many solver iterations per frame tends to create runaway ctrl targets when contacts block motion.
    const iterations = isPaused ? this.config.maxIterations : (this.config.runningIterations ?? 1);

    for (let iter = 0; iter < iterations; iter++) {
      this.mujoco.mj_forward(this.model, this.data);
      this.readCurrentPose();

      if (this.config.lockOrientationOnTranslate && this._holdOrientation && this._lockedOrientationActive) {
        this.targetQuaternion.copy(this._lockedQuaternion).normalize();
      }

      const errorMagnitude = this.computePoseError();
      // Stall guard (target anti-windup):
      const stallMinImprovement = this.config.stallMinImprovement ?? 1e-4;
      const stallMaxIterations = this.config.stallMaxIterations ?? 2;
      if (this._lastSolveError != null) {
        const improvement = this._lastSolveError - errorMagnitude; // positive is good
        if (improvement < stallMinImprovement) {
          this._stallCount += 1;
        } else {
          this._stallCount = 0;
        }
      }
      this._lastSolveError = errorMagnitude;

      if ((stallMaxIterations > 0) && (this._stallCount >= stallMaxIterations)) {
        if (this.config.snapTargetToCurrentOnStall) {
          this.targetPosition.copy(this.currentPose.position);
          this.targetQuaternion.copy(this.currentPose.quaternion).normalize();
        }
        this.targetDirty = false;
        this._stallCount = 0;
        break;
      }
      if (errorMagnitude < 1e-3) {
        if (this.targetDirty) {
          // console.log(`[IKController] Converged after ${iter + 1} iterations, error=${errorMagnitude.toFixed(6)}`);
        }
        this.targetDirty = false;
        this._stallCount = 0;
        break;
      }

      const jacobian = this.computeNumericalJacobian();
      // LM / trust-region least-squares step, similar to the JAX version:
      // (J^T W J + λ I) Δ = J^T W e  (+ joint-limit cost)
      const step = this.computeLevenbergMarquardtStep(jacobian);
      if (!this.applyJointIncrement(step, isPaused)) {
        break;
      }
    }
  }

  readCurrentPose() {
    getPosition(this.data.xpos, this.endEffectorBodyId, this.currentPose.position);
    getQuaternion(this.data.xquat, this.endEffectorBodyId, this.currentPose.quaternion);
    
    // Compute translation error for logging
    const tempError = this.currentPose.position.clone().sub(this.targetPosition);
    
    // Print current EE position (only occasionally to avoid spam)
    if (!this._lastEELog || (Date.now() - this._lastEELog) > 1000) {
      // console.log('[IKController] Current EE position:', {
      //   current: {
      //     x: this.currentPose.position.x.toFixed(4),
      //     y: this.currentPose.position.y.toFixed(4),
      //     z: this.currentPose.position.z.toFixed(4)
      //   },
      //   target: {
      //     x: this.targetPosition.x.toFixed(4),
      //     y: this.targetPosition.y.toFixed(4),
      //     z: this.targetPosition.z.toFixed(4)
      //   },
      //   error: {
      //     x: tempError.x.toFixed(4),
      //     y: tempError.y.toFixed(4),
      //     z: tempError.z.toFixed(4),
      //     magnitude: tempError.length().toFixed(4)
      //   }
      // });
      this._lastEELog = Date.now();
    }
  }

  computePoseError() {
    this.translationError.copy(this.targetPosition).sub(this.currentPose.position);
    const posError = this.translationError.length();

    // IMPORTANT: enforce shortest-arc quaternion distance to avoid 180-degree flips.
    // q and -q represent the same rotation, but without this normalization the solver
    // can jump between equivalent representations near singularities.
    const relative = this.targetQuaternion.clone().multiply(this.currentPose.quaternion.clone().invert());
    if (relative.w < 0) {
      relative.x *= -1;
      relative.y *= -1;
      relative.z *= -1;
      relative.w *= -1;
    }
    const angle = 2 * Math.acos(THREE.MathUtils.clamp(relative.w, -1, 1));
    if (angle < 1e-5) {
      this.rotationError.set(0, 0, 0);
    } else {
      const denom = Math.sqrt(1 - relative.w * relative.w);
      this.axis.set(
        relative.x / denom,
        relative.y / denom,
        relative.z / denom);
      this.rotationError.copy(this.axis).multiplyScalar(angle);
    }

    return posError + this.rotationError.length();
  }

  computeNumericalJacobian() {
    const dofCount = this.controlledDofs.length;
    const jacobian = new Float64Array(dofCount * 6);
    const eps = 1e-4;

    this.qposScratch.set(this.data.qpos);
    const basePosition = this.currentPose.position.clone();
    const baseQuat = this.currentPose.quaternion.clone();

    for (let c = 0; c < dofCount; c++) {
      const addr = this.controlledDofs[c];
      this.data.qpos.set(this.qposScratch);
      this.data.qpos[addr] += eps;
      this.mujoco.mj_forward(this.model, this.data);
      getPosition(this.data.xpos, this.endEffectorBodyId, this.poseScratch.position);
      getQuaternion(this.data.xquat, this.endEffectorBodyId, this.poseScratch.quaternion);

      const posDelta = this.poseScratch.position.clone().sub(basePosition).multiplyScalar(1 / eps);
      const rotDelta = this.computeOrientationDelta(baseQuat, this.poseScratch.quaternion).multiplyScalar(1 / eps);

      jacobian[(0 * dofCount) + c] = posDelta.x;
      jacobian[(1 * dofCount) + c] = posDelta.y;
      jacobian[(2 * dofCount) + c] = posDelta.z;
      jacobian[(3 * dofCount) + c] = rotDelta.x;
      jacobian[(4 * dofCount) + c] = rotDelta.y;
      jacobian[(5 * dofCount) + c] = rotDelta.z;
    }

    // IMPORTANT: restore baseline qpos + kinematics. Finite-diff loop perturbs qpos.
    this.data.qpos.set(this.qposScratch);
    this.mujoco.mj_forward(this.model, this.data);

    return jacobian;
  }

  computeJacobianTransposeStep(jacobian) {
    const dofCount = this.controlledDofs.length;
    const step = new Float64Array(dofCount);
    const errorVec = [
      this.translationError.x * this.config.translationGain,
      this.translationError.y * this.config.translationGain,
      this.translationError.z * this.config.translationGain,
      this.rotationError.x * this.config.rotationGain,
      this.rotationError.y * this.config.rotationGain,
      this.rotationError.z * this.config.rotationGain,
    ];

    for (let c = 0; c < dofCount; c++) {
      let accum = 0;
      for (let r = 0; r < 6; r++) {
        accum += jacobian[(r * dofCount) + c] * errorVec[r];
      }
      step[c] = THREE.MathUtils.clamp(accum, -this.config.stepLimit, this.config.stepLimit);
    }
    return step;
  }

  computeLevenbergMarquardtStep(jacobian) {
    // Pose cost:
    //   minimize || W^(1/2) * (pose_error) ||^2
    // with pose_error = [dx, dy, dz, dRx, dRy, dRz]
    //
    // Limit cost (soft barrier near limits):
    //   add per-DOF residuals when q approaches limits.
    //
    // Solve one LM step:
    //   (J^T W J + J_l^T W_l J_l + λ I) Δ = J^T W e + J_l^T W_l e_l
    const dofCount = this.controlledDofs.length;
    const step = new Float64Array(dofCount);

    // Pose residual e (6,)
    const e = new Float64Array([
      this.translationError.x * this.config.translationGain,
      this.translationError.y * this.config.translationGain,
      this.translationError.z * this.config.translationGain,
      this.rotationError.x * this.config.rotationGain,
      this.rotationError.y * this.config.rotationGain,
      this.rotationError.z * this.config.rotationGain,
    ]);

    // Weights for pose residual
    const wPos = this.config.posWeight ?? 50.0;
    const baseWOri = this.config.oriWeight ?? 10.0;
    const holdWOri = this.config.holdOriWeight ?? 200.0;
    const wOri = (this.config.holdOrientationOnTranslate && this._holdOrientation) ? holdWOri : baseWOri;
    const w = new Float64Array([wPos, wPos, wPos, wOri, wOri, wOri]);

    // Build normal equations: H = J^T W J, g = J^T W e
    const H = new Float64Array(dofCount * dofCount);
    const g = new Float64Array(dofCount);

    for (let i = 0; i < dofCount; i++) {
      let gi = 0;
      for (let r = 0; r < 6; r++) {
        gi += jacobian[(r * dofCount) + i] * (w[r] * e[r]);
      }
      g[i] = gi;

      for (let j = 0; j < dofCount; j++) {
        let hij = 0;
        for (let r = 0; r < 6; r++) {
          hij += jacobian[(r * dofCount) + i] * (w[r] * jacobian[(r * dofCount) + j]);
        }
        H[i * dofCount + j] = hij;
      }
    }

    // Add joint limit cost as diagonal contributions (since limit residual depends on q_i only)
    const limitWeight = this.config.limitWeight ?? 100.0;
    const marginFrac = this.config.limitMarginFraction ?? 0.1;
    if (limitWeight > 0 && marginFrac > 0) {
      for (let k = 0; k < dofCount; k++) {
        const addr = this.controlledDofs[k];
        const jointId = this.qposToJointIdMap.get(addr);
        if (jointId === undefined) continue;
        const range = this.model.jnt_range;
        if (!range || range.length < (jointId + 1) * 2) continue;
        const lo = range[jointId * 2 + 0];
        const hi = range[jointId * 2 + 1];
        if (!(hi > lo)) continue;

        const q = this.data.qpos[addr];
        const span = hi - lo;
        const margin = Math.max(1e-6, span * marginFrac);
        const innerLo = lo + margin;
        const innerHi = hi - margin;

        let residual = 0;
        let deriv = 0; // d(residual)/dq
        if (q < innerLo) {
          residual = (innerLo - q) / margin; // positive
          deriv = -1 / margin;
        } else if (q > innerHi) {
          residual = (q - innerHi) / margin; // positive
          deriv = 1 / margin;
        }

        if (residual !== 0 && deriv !== 0) {
          // Add to H and g:
          // minimize limitWeight * residual^2
          // => H += (limitWeight * deriv^2) on diagonal
          //    g += (limitWeight * deriv * residual)
          H[k * dofCount + k] += limitWeight * (deriv * deriv);
          g[k] += limitWeight * (deriv * residual);
        }
      }
    }

    const jointLimitPenalty = (qposLike) => {
      if (!(limitWeight > 0) || !(marginFrac > 0)) return 0;
      const range = this.model.jnt_range;
      if (!range) return 0;
      let penalty = 0;
      for (let k = 0; k < dofCount; k++) {
        const addr = this.controlledDofs[k];
        const jointId = this.qposToJointIdMap.get(addr);
        if (jointId === undefined) continue;
        if (range.length < (jointId + 1) * 2) continue;
        const lo = range[jointId * 2 + 0];
        const hi = range[jointId * 2 + 1];
        if (!(hi > lo)) continue;

        const q = qposLike[addr];
        const span = hi - lo;
        const margin = Math.max(1e-6, span * marginFrac);
        const innerLo = lo + margin;
        const innerHi = hi - margin;

        let residual = 0;
        if (q < innerLo) residual = (innerLo - q) / margin;
        else if (q > innerHi) residual = (q - innerHi) / margin;
        penalty += limitWeight * residual * residual;
      }
      return penalty;
    };

    // Current cost (pose + limit)
    let currentPoseCost = 0;
    for (let r = 0; r < 6; r++) currentPoseCost += w[r] * e[r] * e[r];
    const currentCost = currentPoseCost + jointLimitPenalty(this.data.qpos);

    const lambdaFactor = this.config.lambdaFactor ?? 2.0;
    const lambdaMin = this.config.lambdaMin ?? 1e-5;
    const lambdaMax = this.config.lambdaMax ?? 1e10;
    const stepQualityMin = this.config.stepQualityMin ?? 1e-3;
    const maxTrials = this.config.lmMaxTrials ?? 6;

    let lambda = this._lmLambda ?? (this.config.lambdaInitial ?? 1.0);
    lambda = THREE.MathUtils.clamp(lambda, lambdaMin, lambdaMax);

    // We'll reuse a candidate qpos buffer
    const qposCandidate = new Float64Array(this.data.qpos.length);

    for (let trial = 0; trial < maxTrials; trial++) {
      // Copy base H/g from current pose+limit linearization (already in H/g arrays)
      // Rebuild fresh each trial because we mutate diagonal with λ.
      const Htrial = new Float64Array(H);
      const gtrial = new Float64Array(g);

      // LM damping λ I
      for (let i = 0; i < dofCount; i++) {
        Htrial[i * dofCount + i] += lambda;
      }

      const delta = this.solveLinearSystem(Htrial, gtrial, dofCount);
      for (let i = 0; i < dofCount; i++) {
        step[i] = THREE.MathUtils.clamp(delta[i], -this.config.stepLimit, this.config.stepLimit);
      }

      // Predicted pose residual: e - J*Δ
      let predictedPoseCost = 0;
      for (let r = 0; r < 6; r++) {
        let jstep = 0;
        for (let c = 0; c < dofCount; c++) {
          jstep += jacobian[(r * dofCount) + c] * step[c];
        }
        const ep = e[r] - jstep;
        predictedPoseCost += w[r] * ep * ep;
      }

      // Candidate qpos = current qpos + step (on controlled dofs)
      qposCandidate.set(this.data.qpos);
      for (let i = 0; i < dofCount; i++) {
        const addr = this.controlledDofs[i];
        qposCandidate[addr] = qposCandidate[addr] + step[i];
      }
      const predictedCost = predictedPoseCost + jointLimitPenalty(qposCandidate);

      // Actual proposed cost (evaluate FK at qposCandidate, no side-effects)
      this.qposScratch.set(this.data.qpos);
      this.data.qpos.set(qposCandidate);
      this.mujoco.mj_forward(this.model, this.data);

      getPosition(this.data.xpos, this.endEffectorBodyId, this.poseScratch.position);
      getQuaternion(this.data.xquat, this.endEffectorBodyId, this.poseScratch.quaternion);

      const te = this.targetPosition.clone().sub(this.poseScratch.position);
      const relative = this.targetQuaternion.clone().multiply(this.poseScratch.quaternion.clone().invert());
      if (relative.w < 0) {
        relative.x *= -1; relative.y *= -1; relative.z *= -1; relative.w *= -1;
      }
      const angle = 2 * Math.acos(THREE.MathUtils.clamp(relative.w, -1, 1));
      let rx = 0, ry = 0, rz = 0;
      if (angle >= 1e-5) {
        const denom = Math.sqrt(1 - relative.w * relative.w);
        const ax = relative.x / denom;
        const ay = relative.y / denom;
        const az = relative.z / denom;
        rx = ax * angle;
        ry = ay * angle;
        rz = az * angle;
      }
      const proposedE = new Float64Array([
        te.x * this.config.translationGain,
        te.y * this.config.translationGain,
        te.z * this.config.translationGain,
        rx * this.config.rotationGain,
        ry * this.config.rotationGain,
        rz * this.config.rotationGain,
      ]);
      let proposedPoseCost = 0;
      for (let r = 0; r < 6; r++) proposedPoseCost += w[r] * proposedE[r] * proposedE[r];
      const proposedCost = proposedPoseCost + jointLimitPenalty(qposCandidate);

      // Restore live state
      this.data.qpos.set(this.qposScratch);
      this.mujoco.mj_forward(this.model, this.data);

      const denom = (predictedCost - currentCost);
      const denomTiny = Math.abs(denom) < 1e-12;
      const rho = denomTiny ? 0.0 : (proposedCost - currentCost) / denom;
      const accept = (proposedCost < currentCost) && (denomTiny || (rho >= stepQualityMin));

      if (accept) {
        lambda = Math.max(lambdaMin, lambda / lambdaFactor);
        this._lmLambda = lambda;
        return step;
      }

      lambda = Math.min(lambdaMax, lambda * lambdaFactor);
      this._lmLambda = lambda;
    }

    return new Float64Array(dofCount);
  }

  solveLinearSystem(A, b, n) {
    // Solve A x = b for n x n A using Gauss-Jordan elimination.
    // A is Float64Array(n*n) row-major.
    const M = new Float64Array(n * (n + 1));
    for (let r = 0; r < n; r++) {
      for (let c = 0; c < n; c++) {
        M[r * (n + 1) + c] = A[r * n + c];
      }
      M[r * (n + 1) + n] = b[r];
    }

    for (let col = 0; col < n; col++) {
      // Pivot
      let pivotRow = col;
      let pivotVal = Math.abs(M[pivotRow * (n + 1) + col]);
      for (let r = col + 1; r < n; r++) {
        const v = Math.abs(M[r * (n + 1) + col]);
        if (v > pivotVal) {
          pivotVal = v;
          pivotRow = r;
        }
      }
      if (pivotVal < 1e-12) {
        return new Float64Array(n);
      }
      if (pivotRow !== col) {
        for (let c = col; c < n + 1; c++) {
          const tmp = M[col * (n + 1) + c];
          M[col * (n + 1) + c] = M[pivotRow * (n + 1) + c];
          M[pivotRow * (n + 1) + c] = tmp;
        }
      }

      // Normalize pivot row
      const pivot = M[col * (n + 1) + col];
      for (let c = col; c < n + 1; c++) {
        M[col * (n + 1) + c] /= pivot;
      }

      // Eliminate
      for (let r = 0; r < n; r++) {
        if (r === col) continue;
        const factor = M[r * (n + 1) + col];
        if (Math.abs(factor) < 1e-12) continue;
        for (let c = col; c < n + 1; c++) {
          M[r * (n + 1) + c] -= factor * M[col * (n + 1) + c];
        }
      }
    }

    const x = new Float64Array(n);
    for (let r = 0; r < n; r++) {
      x[r] = M[r * (n + 1) + n];
    }
    return x;
  }

  applyJointIncrement(step, isPaused = false) {
    if (!step || !step.length) { return false; }
    
    // Always use actuator controls in simulation mode
    // Never fallback to direct qpos setting - this causes conflicts with physics engine
    for (let i = 0; i < this.controlledDofs.length; i++) {
      const addr = this.controlledDofs[i];
      const actuatorIdx = this.qposToActuatorMap.get(addr);
      
      if (actuatorIdx === undefined || actuatorIdx < 0 || actuatorIdx >= this.data.ctrl.length) {
        // Find joint name for better error message
        let jointName = 'unknown';
        for (let j = 0; j < this.model.njnt; j++) {
          if (this.model.jnt_qposadr[j] === addr) {
            jointName = this.decodeName(this.model.name_jntadr[j]);
            break;
          }
        }
        // console.error(`[IKController] ERROR: No valid actuator found for joint "${jointName}" qpos[${addr}]`);
        throw new Error(`[IKController] No actuator found for joint "${jointName}" qpos[${addr}]. Cannot proceed.`);
      }
      
      // Get current joint position and add step
      const currentQpos = this.data.qpos[addr];
      let targetQpos = currentQpos + step[i];

      // Clamp to actuator ctrlrange if available (prevents "drift" into unreachable space)
      const ctrlRange = this.model.actuator_ctrlrange;
      if (ctrlRange && ctrlRange.length >= (actuatorIdx + 1) * 2) {
        const lo = ctrlRange[actuatorIdx * 2 + 0];
        const hi = ctrlRange[actuatorIdx * 2 + 1];
        // Some models leave ctrlrange at 0,0 when not limited; ignore that.
        if (hi > lo) {
          targetQpos = THREE.MathUtils.clamp(targetQpos, lo, hi);
        }
      }

      // Clamp to joint range as well (hard safety to avoid drifting into joint limits)
      const jointId = this.qposToJointIdMap.get(addr);
      const jntRange = this.model.jnt_range;
      if (jointId !== undefined && jntRange && jntRange.length >= (jointId + 1) * 2) {
        const jlo = jntRange[jointId * 2 + 0];
        const jhi = jntRange[jointId * 2 + 1];
        if (jhi > jlo) {
          targetQpos = THREE.MathUtils.clamp(targetQpos, jlo, jhi);
        }
      }

      // Anti-windup clamp around current qpos (prevents ctrl targets from running away under contact)
      const maxOffset = this.config.maxCtrlOffset ?? 0.25;
      if (maxOffset > 0) {
        targetQpos = THREE.MathUtils.clamp(targetQpos, currentQpos - maxOffset, currentQpos + maxOffset);
      }
      
      // Set actuator control to target position
      this.data.ctrl[actuatorIdx] = targetQpos;
    }
    
    return true;
  }

  computeOrientationDelta(baseQuat, targetQuat) {
    const relative = targetQuat.clone().multiply(baseQuat.clone().invert());
    if (relative.w < 0) {
      relative.x *= -1;
      relative.y *= -1;
      relative.z *= -1;
      relative.w *= -1;
    }
    const sinHalf = Math.sqrt(relative.x * relative.x + relative.y * relative.y + relative.z * relative.z);
    if (sinHalf < 1e-9) {
      return new THREE.Vector3(0, 0, 0);
    }
    const axis = new THREE.Vector3(relative.x / sinHalf, relative.y / sinHalf, relative.z / sinHalf);
    const angle = 2.0 * Math.atan2(sinHalf, relative.w);
    return axis.multiplyScalar(angle);
  }

  applyMocapTarget() {
    const mocapIndex = this.mocapId;
    if (mocapIndex < 0) { return; }
    const posAddr = mocapIndex * 3;
    const quatAddr = mocapIndex * 4;
    const mujocoPos = toMujocoPos(this.targetPosition.clone());
    const mujocoQuat = toMujocoQuat(this.targetQuaternion.clone());

    const posBuffer = this.data.mocap_pos;
    posBuffer[posAddr + 0] = mujocoPos.x;
    posBuffer[posAddr + 1] = mujocoPos.y;
    posBuffer[posAddr + 2] = mujocoPos.z;

    const quatBuffer = this.data.mocap_quat;
    quatBuffer[quatAddr + 0] = mujocoQuat.w;
    quatBuffer[quatAddr + 1] = mujocoQuat.x;
    quatBuffer[quatAddr + 2] = mujocoQuat.y;
    quatBuffer[quatAddr + 3] = mujocoQuat.z;

    this.targetDirty = false;
  }
}
