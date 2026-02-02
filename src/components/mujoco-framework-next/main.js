import * as THREE           from 'three';
import { drawTendonsAndFlex, getPosition, getQuaternion, toMujocoPos, standardNormal } from './mujocoUtils.js';
import {
  getTrajectoryFetchURL,
  startReplayTrajectory,
  loadTrajectoryFromData,
  stopReplayTrajectory,
  updateReplay,
  applyTrajectorySample,
  recordEpisodeSample,
  saveTrajectory,
  serializeTrajectory,
  buildEpisodeMetadata,
} from './utils/trajectoryManager.js';
import {
  refreshNameBuffer,
  readNameAt,
  findIdByName,
  getJointAddress,
} from './utils/nameLookup.js';
import { resolveRobotConfig } from './utils/robot_config.js';
import {
  setupIKController,
  setupGripperControls,
  alignMovementToBase,
  handleTeleopTranslate,
  handleTeleopRotate,
  setGripperState,
  setupKeyboardControls,
} from './utils/teleopController.js';
import {
  setupCheckers,
  updateCheckers,
  getCheckerStatuses,
  maybeFinalizeSuccess,
} from './utils/checkerRuntime.js';
import {
  applyInitialState,
  applyDomainRandomization,
} from './utils/stateRuntime.js';
import { updateSuccessTracking } from './utils/successTracker.js';
import {
  broadcastStatus,
  startStatusBroadcasting,
  stopStatusBroadcasting,
} from './utils/statusBroadcaster.js';
import { setupRenderer } from './utils/rendererRuntime.js';
import { initDemo, setupReplay, onModelReloaded } from './utils/initRuntime.js';

const initialScene = "scenes/close_box.xml";

export class MuJoCoDemo {
  constructor(options = {}) {
    this.mujoco = null;
    this.model = null;
    this.data = null;
    this.params = { scene: initialScene, paused: false, help: false, ctrlnoiserate: 0.0, ctrlnoisestd: 0.0, keyframeNumber: 0 };
    this.mujoco_time = 0.0;
    this.bodies  = {}, this.lights = {};
    this.tmpVec  = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.updateGUICallbacks = [];
    this.episodes = [];
    this.currentEpisode = [];
    this.gripperClosed = false;
    this.gripperFingerJointNames = [];
    this.gripperFingerJointAddresses = [];
    this.gripperFingerActuatorIndices = [];
    this.gripperOpenPose = 0.04;
    this.gripperClosedPose = 0.0;
    this.lastEpisodeSample = 0;
    this.episodeStartTime = null;
    this.episodeStartWallClock = null;
    this.episodeSampleIntervalMs = 50;
    this.decimation = 4;
    this.episodeStepCount = 0;
    this.checkpointState = null;
    this.checkpointSavedAt = null;
    this.textDecoder = new TextDecoder('utf-8');
    this.namesBuffer = null;
    // Drag interaction is intentionally disabled (user request).
    // Keep the field for backwards compatibility with older codepaths.
    this.dragStateManager = null;
    this.replaySamples = null;
    // Checker progress logging (step-based, extensible across tasks)
    this.checkerProgressPrintEverySteps = 10; // 每隔多少 step 打印一次子目标进度
    this._lastCheckerProgressLine = '';
    this.replayIndex = 0;
    this.replayStartTime = null;
    this.replayPlaying = false;
    this.lastSavedTrajectoryPath = null;
    // Loading progress callback (used by React pages to show progress UI)
    this.onLoadingProgress = options.onLoadingProgress || null;
    // Dynamic checker config (provided by React page from backend task config)
    this.checkerConfig = options.checkerConfig || null;
    this.checkerManager = null;
    // Dynamic initial state (provided by React page from backend task config)
    this.initialStateConfig = options.initialState || null;
    this.domainRandomizationConfig = options.domainRandomization ?? null;
    this.checkerUpdateInterval = 1000;
    this.lastCheckerUpdate = 0;
    this.successStartTime = null;
    this.successAchieved = false;
    this.successTime = null;
    this.successExported = false; // ensure we export trajectory only once per success
    this.pendingReplayFlag = false;
    this.pendingReplaySamples = null;
    this.robotConfig = null;
    this.robotControlConfig = {
      endEffectorBodyName: null,
      jointNames: [],
      mode: 'joint',
      translationGain: 1.0,
      rotationGain: 0.5,
      maxIterations: 6,
      stepLimit: 0.04,
    };
    this.initialState = null;
    this.guiPauseController = null;
    this.isSavingEpisode = false;
    this.modelXml = null;
    
    // Callbacks for React integration (instead of postMessage)
    this.onStatusUpdate = null; // (status) => void
    this.onTrajectoryExport = null; // (trajectory, metadata) => void
    this.statusBroadcastInterval = null;

    // Allow embedding into a custom parent element (for React/Next.js integration).
    // Falls back to document.body for legacy/static usage.
    const parentElement = options.parentElement || options.parent || document.body;
    this.container = document.createElement('div');
    // Ensure container takes full size of parent and doesn't break out
    this.container.style.position = 'relative';
    this.container.style.width = '100%';
    this.container.style.height = '100%';
    this.container.style.overflow = 'hidden';
    this.container.style.display = 'block';
    parentElement.appendChild(this.container);

    setupRenderer(this, parentElement);

    setupKeyboardControls(this);
  }

  toggleGripper() {
    this.setGripperState(!this.gripperClosed);
  }

  emitLoadingProgress(update) {
    if (typeof this.onLoadingProgress !== "function") return;
    try {
      this.onLoadingProgress(update);
    } catch (e) {
      // Never break init because UI progress handler threw
      console.warn("[MuJoCoDemo] onLoadingProgress handler threw:", e);
    }
  }

  async init() {
    return initDemo(this, initialScene);
  }

  setupReplay() {
    return setupReplay(this);
  }

  onModelReloaded() {
    return onModelReloaded(this);
  }

  setupCheckers() {
    return setupCheckers(this);
  }

  setupIKController() {
    return setupIKController(this);
  }

  refreshNameBuffer() {
    return refreshNameBuffer(this);
  }

  readNameAt(address) {
    return readNameAt(this, address);
  }

  findIdByName(name, addressArray, count) {
    return findIdByName(this, name, addressArray, count);
  }

  getJointAddress(name) {
    return getJointAddress(this, name);
  }

  setupGripperControls() {
    return setupGripperControls(this);
  }

  setupRobotConfig() {
    if (!this.model) { return; }
    const resolved = resolveRobotConfig({
      xmlText: this.modelXml,
      model: this.model,
      readNameAt: this.readNameAt.bind(this),
    });
    this.robotConfig = resolved;
    this.robotControlConfig = resolved.controlConfig;
    if (resolved.gripper?.fingerJointNames) {
      this.gripperFingerJointNames = resolved.gripper.fingerJointNames.slice();
    }
    if (typeof resolved.gripper?.openPose === 'number') {
      this.gripperOpenPose = resolved.gripper.openPose;
    }
    if (typeof resolved.gripper?.closedPose === 'number') {
      this.gripperClosedPose = resolved.gripper.closedPose;
    }
  }

  // ========== Task Initial State (data-driven) ==========
  applyInitialState(initialState) {
    return applyInitialState(this, initialState);
  }

  applyDomainRandomization(domainRandomization) {
    return applyDomainRandomization(this, domainRandomization);
  }

  setRobotReadyPose() {
    if (!this.model || !this.data) { return; }
    const readyPose = this.robotConfig?.readyPose;
    if (!readyPose?.jointNames?.length || !readyPose?.qpos?.length) {
      return;
    }
    for (let i = 0; i < readyPose.jointNames.length && i < readyPose.qpos.length; i++) {
      const jointName = readyPose.jointNames[i];
      const address = this.getJointAddress(jointName);
      if (address >= 0 && address < this.data.qpos.length) {
        const targetQpos = readyPose.qpos[i];
        this.data.qpos[address] = targetQpos;
        const actuatorIdx = this.findIdByName(jointName, this.model.name_actuatoradr, this.model.nu);
        if (actuatorIdx >= 0 && actuatorIdx < this.data.ctrl.length) {
          this.data.ctrl[actuatorIdx] = targetQpos;
        } else {
          for (let a = 0; a < this.model.nu; a++) {
            const actuatorName = this.readNameAt(this.model.name_actuatoradr[a]);
            if (actuatorName === jointName ||
                actuatorName.endsWith('/' + jointName) ||
                actuatorName.endsWith(jointName)) {
              this.data.ctrl[a] = targetQpos;
              break;
            }
          }
        }
      }
    }
    this.mujoco.mj_forward(this.model, this.data);
    this.updateSceneFromData();
  }

  alignMovementToBase(delta) {
    return alignMovementToBase(delta);
  }

  handleTeleopTranslate(delta) {
    return handleTeleopTranslate(this, delta);
  }

  handleTeleopRotate(deltaEuler) {
    return handleTeleopRotate(this, deltaEuler);
  }

  setGripperState(closed) {
    return setGripperState(this, closed);
  }

  async toggleReplayTrajectory() {
    if (this.replayPlaying) {
      this.stopReplayTrajectory();
      return;
    }
    await this.startReplayTrajectory();
  }

  async startReplayTrajectory() {
    return startReplayTrajectory(this);
  }

  loadTrajectoryFromData(samples) {
    return loadTrajectoryFromData(this, samples);
  }

  stopReplayTrajectory() {
    return stopReplayTrajectory(this);
  }

  getTrajectoryFetchURL(path) {
    return getTrajectoryFetchURL(path);
  }

  saveCheckpoint() {
    const snapshot = this.createStateSnapshot();
    if (!snapshot) { return; }
    this.checkpointState = {
      state: snapshot,
      episode: this.currentEpisode.slice(),
      episodeStartTime: this.episodeStartTime,
      episodeStartWallClock: this.episodeStartWallClock,
      lastEpisodeSample: this.lastEpisodeSample,
      episodeStepCount: this.episodeStepCount,
      step: this.currentEpisode.length,
      timestamp: Date.now()
    };
    this.checkpointSavedAt = Date.now();
  }

  restoreCheckpoint() {
    if (!this.checkpointState) { return; }
    const stateToRestore = this.checkpointState.state || this.checkpointState;
    this.applyStateSnapshot(stateToRestore);
    if (this.checkpointState.episode) {
      this.currentEpisode = this.checkpointState.episode.slice();
      this.episodeStartTime = this.checkpointState.episodeStartTime;
      this.episodeStartWallClock = this.checkpointState.episodeStartWallClock;
      this.lastEpisodeSample = this.checkpointState.lastEpisodeSample;
      this.episodeStepCount = this.checkpointState.episodeStepCount || 0;
    } else {
      this.clearEpisodeState();
    }
    if (this.ikController) {
      this.ikController.syncCtrlFromQpos();
      this.ikController.syncTargetsFromModel();
    }
    this.mujoco.mj_forward(this.model, this.data);
    this.updateSceneFromData();
    this.params.paused = true;
    this.guiPauseController?.setValue(true);
    this.keyboardStateManager?.clearActiveKeys?.();
    this.keyboardStateManager?.clearPressedKeys?.();
  }

  updateReplay(timeMS) {
    return updateReplay(this, timeMS);
  }

  applyTrajectorySample(sample) {
    return applyTrajectorySample(this, sample);
  }

  async completeEpisode() {
    if (this.isSavingEpisode) { return; }
    this.recordEpisodeSample(performance.now(), true);
    if (!this.currentEpisode.length) { return; }
    const samples = this.currentEpisode.slice();
    const metadata = this.buildEpisodeMetadata(samples);
    const payload = { version: 1, metadata, samples };
    let savedLocation = null;
    try {
      this.isSavingEpisode = true;
      const result = await this.saveTrajectory(payload);
      if (result) {
        savedLocation = result.path || result.filename || null;
        if (savedLocation) {
          payload.metadata.savedTo = savedLocation;
          this.lastSavedTrajectoryPath = savedLocation;
        }
      }
    } catch (error) {
      console.error('[Teleop] Failed to persist trajectory:', error);
    } finally {
      this.isSavingEpisode = false;
    }
    this.episodes.push(payload);
    this.clearEpisodeState();
    this.setRobotReadyPose();
    this.captureInitialState();
    this.resetSimulation();
    this.setRobotReadyPose();
    this.captureInitialState();
    this.keyboardStateManager?.clearActiveKeys?.();
    this.keyboardStateManager?.clearPressedKeys?.();
  }

  resetEpisode() {
    this.clearEpisodeState();
    this.keyboardStateManager?.clearActiveKeys?.();
    this.keyboardStateManager?.clearPressedKeys?.();
    this.resetSuccessTimer();
    this.resetSimulation();
  }

  saveAndExit() {
  }

  resetSuccessTimer() {
    this.successStartTime = performance.now();
    this.successAchieved = false;
    this.successTime = null;
    this.successExported = false;
  }

  maybeFinalizeSuccess(timeMS) {
    return maybeFinalizeSuccess(this, timeMS);
  }

  updateSuccessTracking() {
    return updateSuccessTracking(this);
  }

  resetSimulation() {
    this.restoreInitialState();
    if (this.domainRandomizationConfig) {
      this.applyDomainRandomization(this.domainRandomizationConfig);
    }
    this.mujoco.mj_forward(this.model, this.data);
    this.updateSceneFromData();
    if (this.ikController) {
      this.ikController.syncCtrlFromQpos();
      // Also reset IK target orientation / locks when resetting the scene.
      if (this.ikController.resetTargetsFromModel) {
        this.ikController.resetTargetsFromModel();
      } else {
        this.ikController.syncTargetsFromModel();
      }
    } else {
      const jointNames = [
        ...(this.robotControlConfig?.jointNames || []),
        ...(this.gripperFingerJointNames || []),
      ];
      for (const jointName of jointNames) {
        const address = this.getJointAddress(jointName);
        if (address >= 0 && address < this.data.qpos.length) {
          const qposValue = this.data.qpos[address];
          const actuatorIdx = this.findIdByName(jointName, this.model.name_actuatoradr, this.model.nu);
          if (actuatorIdx >= 0 && actuatorIdx < this.data.ctrl.length) {
            this.data.ctrl[actuatorIdx] = qposValue;
          } else {
            for (let a = 0; a < this.model.nu; a++) {
              const actuatorName = this.readNameAt(this.model.name_actuatoradr[a]);
              if (actuatorName === jointName || 
                  actuatorName.endsWith('/' + jointName) ||
                  actuatorName.endsWith(jointName)) {
                this.data.ctrl[a] = qposValue;
                break;
              }
            }
          }
        }
      }
    }
    this.mujoco_time = performance.now();
  }

  recordEpisodeSample(timeMS, force = false) {
    return recordEpisodeSample(this, timeMS, force);
  }

  async saveTrajectory(payload) {
    return saveTrajectory(payload);
  }

  clearEpisodeState() {
    this.currentEpisode = [];
    this.lastEpisodeSample = 0;
    this.episodeStartTime = null;
    this.episodeStartWallClock = null;
    this.resetSuccessTimer();
  }

  createStateSnapshot() {
    if (!this.model || !this.data) { return null; }
    return {
      qpos: Array.from(this.data.qpos),
      qvel: Array.from(this.data.qvel),
      ctrl: this.data.ctrl?.length ? Array.from(this.data.ctrl) : null,
      mocap_pos: this.model.nmocap > 0 ? Array.from(this.data.mocap_pos) : null,
      mocap_quat: this.model.nmocap > 0 ? Array.from(this.data.mocap_quat) : null,
    };
  }

  applyStateSnapshot(snapshot) {
    if (!snapshot || !this.data) { return; }
    this.data.qpos.set(snapshot.qpos);
    this.data.qvel.set(snapshot.qvel);
    if (snapshot.ctrl && this.data.ctrl?.length) {
      this.data.ctrl.set(snapshot.ctrl);
    }
    if (snapshot.mocap_pos && this.model.nmocap > 0) {
      this.data.mocap_pos.set(snapshot.mocap_pos);
    }
    if (snapshot.mocap_quat && this.model.nmocap > 0) {
      this.data.mocap_quat.set(snapshot.mocap_quat);
    }
  }

  captureInitialState() {
    try {
      if (this.mujoco && this.model && this.data) {
        this.mujoco.mj_forward(this.model, this.data);
      }
      if (this.ikController) {
        this.ikController.syncCtrlFromQpos();
        this.ikController.syncTargetsFromModel();
      }
    } catch (e) {
      console.warn('[Teleop] captureInitialState: failed to sync ctrl/targets:', e);
    }
    this.initialState = this.createStateSnapshot();
  }

  restoreInitialState() {
    if (!this.mujoco || !this.model || !this.data) {
      return;
    }
    if (!this.initialState) {
      this.mujoco.mj_resetData(this.model, this.data);
      return;
    }

    this.applyStateSnapshot(this.initialState);
  }

  updateSceneFromData() {
    if (!this.model || !this.data) { return; }

    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition(this.data.xpos, b, this.bodies[b].position);
        getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.data.light_xpos, l, this.lights[l].position);
        getPosition(this.data.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    drawTendonsAndFlex(this.mujocoRoot, this.model, this.data);
  }

  onWindowResize() {
    // Use container dimensions instead of window dimensions
    const containerWidth = this.container.clientWidth || window.innerWidth;
    const containerHeight = this.container.clientHeight || window.innerHeight;
    
    this.camera.aspect = containerWidth / containerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize( containerWidth, containerHeight );
  }

  render(timeMS) {
    if (!this.model || !this.data) {
      this.controls.update();
      this.renderer.render( this.scene, this.camera );
      return;
    }
    if (this.replayPlaying) {
      if (!this.replayLoopLogged) {
        console.log(`[Teleop] Entering replay loop with ${this.replaySamples?.length || 0} samples`);
        this.replayLoopLogged = true;
      }
      this.keyboardStateManager?.clearActiveKeys?.();
      this.keyboardStateManager?.clearPressedKeys?.();
      this.updateReplay(timeMS);
      this.updateSceneFromData();
      this.renderer.render( this.scene, this.camera );
      return;
    }
    this.controls.update();
    this.keyboardStateManager?.update(timeMS);
    try {
      this.ikController?.update(timeMS, this.params["paused"]);
    } catch (error) {
      console.error('[Render Loop] IK Controller update error:', error);
      this.ikController = null;
    }
    if (!this.params["paused"]) {
      let timestep = this.model.opt.timestep;
      if (timeMS - this.mujoco_time > 35.0) { this.mujoco_time = timeMS; }
      while (this.mujoco_time < timeMS) {
        if (this.params["ctrlnoisestd"] > 0.0) {
          let rate  = Math.exp(-timestep / Math.max(1e-10, this.params["ctrlnoiserate"]));
          let scale = this.params["ctrlnoisestd"] * Math.sqrt(1 - rate * rate);
          let currentCtrl = this.data.ctrl;
          for (let i = 0; i < currentCtrl.length; i++) {
            currentCtrl[i] = rate * currentCtrl[i] + scale * standardNormal();
            this.params["Actuator " + i] = currentCtrl[i];
          }
        }
        for (let i = 0; i < this.data.qfrc_applied.length; i++) { this.data.qfrc_applied[i] = 0.0; }
        this.mujoco.mj_step(this.model, this.data);
        this.mujoco_time += timestep * 1000.0;
      }
      const stepInfo = this.recordEpisodeSample(timeMS);
      if (stepInfo?.didStep) {
        this.updateSuccessTracking();
      }
      // If the checker has transitioned to success, auto-export the trajectory once.
      this.maybeFinalizeSuccess(timeMS);
    } else if (this.params["paused"]) {
      this.mujoco.mj_forward(this.model, this.data);
      if (this.keyboardStateManager?.hasActiveCommands() || this.currentEpisode.length || this.ikController?.targetDirty) {
        const stepInfo = this.recordEpisodeSample(timeMS);
        if (stepInfo?.didStep) {
          this.updateSuccessTracking();
        }
      }
    }
    this.updateSceneFromData();
    this.updateCheckers(timeMS);
    this.renderer.render( this.scene, this.camera );
  }
  
  updateCheckers(timeMS) {
    return updateCheckers(this, timeMS);
  }
  
  getCheckerStatuses() {
    return getCheckerStatuses(this);
  }

  // ========== React Integration Methods ==========
  
  /**
   * Broadcast current status to React parent via callback
   * Similar to HTML bridge's sendStatus() function
   */
  broadcastStatus() {
    return broadcastStatus(this);
  }

  /**
   * Start broadcasting status updates periodically
   */
  startStatusBroadcasting() {
    return startStatusBroadcasting(this);
  }

  /**
   * Stop broadcasting status updates
   */
  stopStatusBroadcasting() {
    return stopStatusBroadcasting(this);
  }

  /**
   * Serialize current episode trajectory to API format
   * Similar to HTML bridge's serializeTrajectory() function
   */
  serializeTrajectory() {
    return serializeTrajectory(this);
  }

  /**
   * Build episode metadata for trajectory export
   */
  buildEpisodeMetadata(episode) {
    return buildEpisodeMetadata(this, episode);
  }

  /**
   * Public method: Reset simulation
   * Similar to HTML bridge's reset command
   */
  reset() {
    console.log("[MuJoCoDemo] Reset command received");
    // Reset episode state
    this.clearEpisodeState();
    // Reset success flags
    this.successAchieved = false;
    this.successTime = null;
    this.resetSuccessTimer();
    // Reset simulation
    this.resetSimulation();
    // Unpause
    this.params.paused = false;
    if (this.guiPauseController) {
      this.guiPauseController.setValue(false);
    }
    console.log("[MuJoCoDemo] Simulation reset complete");
  }
}

if (typeof window !== 'undefined') {
  window.addEventListener('error', (event) => {
    console.error('[Global Error Handler]', event.error);
  });

  window.addEventListener('unhandledrejection', (event) => {
    console.error('[Unhandled Promise Rejection]', event.reason);
  });
}
