import * as THREE from 'three';
import { IKController } from './IKController.js';
import { KeyboardStateManager } from './KeyboardStateManager.js';

function setupIKController(demo) {
  const ikType = demo.robotConfig?.ik?.type ?? 'numeric';
  if (ikType === 'none') {
    demo.ikController = null;
    return;
  }
  const allBodyNames = [];
  for (let i = 0; i < demo.model.nbody; i++) {
    allBodyNames.push(demo.readNameAt(demo.model.name_bodyadr[i]));
  }
  const allJointNames = [];
  for (let i = 0; i < demo.model.njnt; i++) {
    allJointNames.push(demo.readNameAt(demo.model.name_jntadr[i]));
  }
  const configuredEE = demo.robotControlConfig?.endEffectorBodyName;
  if (!configuredEE) {
    console.warn('[Teleop] No end effector configured; IK disabled.');
    demo.ikController = null;
    return;
  }
  if (!allBodyNames.includes(configuredEE)) {
    throw new Error(`Required end effector body "${configuredEE}" not found in model.`);
  }
  const configuredJoints = demo.robotControlConfig?.jointNames || [];
  const existingJoints = configuredJoints.filter(j => allJointNames.includes(j));
  if (existingJoints.length === 0) {
    const jointsWithActuators = [];
    for (const jointName of allJointNames) {
      let hasActuator = false;
      for (let a = 0; a < demo.model.nu; a++) {
        const actuatorName = demo.readNameAt(demo.model.name_actuatoradr[a]);
        if (actuatorName === jointName || actuatorName.endsWith('/' + jointName) || actuatorName.endsWith(jointName)) {
          hasActuator = true;
          break;
        }
      }
      if (hasActuator) {
        jointsWithActuators.push(jointName);
      }
    }
    if (jointsWithActuators.length > 0) {
      demo.robotControlConfig.jointNames = jointsWithActuators.sort();
    } else {
      throw new Error('No joints with actuators found!');
    }
  } else if (existingJoints.length < configuredJoints.length) {
    demo.robotControlConfig.jointNames = existingJoints;
  }
  try {
    if (ikType !== 'numeric') {
      throw new Error(`Unsupported IK type "${ikType}"`);
    }
    demo.ikController = new IKController(demo.mujoco, demo.model, demo.data, demo.robotControlConfig);
  } catch (error) {
    console.error('[Teleop] Failed to create IKController:', error);
    demo.ikController = null;
  }
}

function setupGripperControls(demo) {
  if (!demo.model) { return; }
  demo.gripperFingerJointAddresses = [];
  demo.gripperFingerActuatorIndices = [];
  for (const jointName of demo.gripperFingerJointNames) {
    const address = demo.getJointAddress(jointName);
    if (address >= 0) {
      demo.gripperFingerJointAddresses.push(address);
    }
    const actuatorIdx = demo.findIdByName(jointName, demo.model.name_actuatoradr, demo.model.nu);
    if (actuatorIdx >= 0 && actuatorIdx < demo.model.nu) {
      demo.gripperFingerActuatorIndices.push(actuatorIdx);
    } else {
      for (let a = 0; a < demo.model.nu; a++) {
        const actuatorName = demo.readNameAt(demo.model.name_actuatoradr[a]);
        if (actuatorName === jointName ||
            actuatorName.endsWith('/' + jointName) ||
            actuatorName.endsWith(jointName)) {
          demo.gripperFingerActuatorIndices.push(a);
          break;
        }
      }
    }
  }
}

function alignMovementToBase(delta) {
  return new THREE.Vector3(delta.x, delta.z, -delta.y);
}

function handleTeleopTranslate(demo, delta) {
  if (demo.ikController) {
    const alignedDelta = alignMovementToBase(delta);
    demo.ikController.adjustTargetPosition(alignedDelta);
  }
}

function handleTeleopRotate(demo, deltaEuler) {
  if (demo.ikController) {
    demo.ikController.adjustTargetOrientation(deltaEuler);
  }
}

function setGripperState(demo, closed) {
  if (!demo.model || !demo.data) { return; }
  if (demo.gripperClosed === closed) { return; }
  demo.gripperClosed = closed;
  const poseValue = closed ? demo.gripperClosedPose : demo.gripperOpenPose;
  // IMPORTANT:
  // - Prefer actuator control (data.ctrl). This allows contact constraints to prevent penetration.
  // - Writing qpos directly will "teleport" fingers through objects and causes gripper penetration.
  const hasActuators = demo.gripperFingerActuatorIndices.length > 0;
  if (hasActuators) {
    for (let i = 0; i < demo.gripperFingerActuatorIndices.length; i++) {
      const actuatorIdx = demo.gripperFingerActuatorIndices[i];
      if (actuatorIdx >= 0 && actuatorIdx < demo.data.ctrl.length) {
        demo.data.ctrl[actuatorIdx] = poseValue;
      }
    }
  } else {
    // Fallback only if actuators are missing (should be rare for Franka models)
    for (const address of demo.gripperFingerJointAddresses) {
      if (address >= 0 && address < demo.data.qpos.length) {
        demo.data.qpos[address] = poseValue;
      }
    }
  }
  demo.mujoco.mj_forward(demo.model, demo.data);
}

function setupKeyboardControls(demo) {
  demo.keyboardStateManager = new KeyboardStateManager({
    // Teleop speed tuning:
    // - translationSpeed: meters per second (0.15 = 15cm/s, feels controllable)
    // - rotationSpeed: radians per second
    translationSpeed: 0.4,
    rotationSpeed: THREE.MathUtils.degToRad(45) * 3,
    onTranslate: demo.handleTeleopTranslate.bind(demo),
    onRotate: demo.handleTeleopRotate.bind(demo),
    // Space toggles gripper open/close (no need to hold)
    gripperMode: 'toggle',
    onGripperCommand: demo.toggleGripper.bind(demo),
    onCompleteEpisode: demo.completeEpisode.bind(demo),
    onResetEpisode: demo.resetEpisode.bind(demo),
    onSaveAndExit: demo.saveAndExit.bind(demo),
    onSaveCheckpoint: demo.saveCheckpoint.bind(demo),
    onRestoreCheckpoint: demo.restoreCheckpoint.bind(demo),
    onReplayTrajectory: demo.toggleReplayTrajectory.bind(demo),
  });
}

export {
  setupIKController,
  setupGripperControls,
  alignMovementToBase,
  handleTeleopTranslate,
  handleTeleopRotate,
  setGripperState,
  setupKeyboardControls,
};
