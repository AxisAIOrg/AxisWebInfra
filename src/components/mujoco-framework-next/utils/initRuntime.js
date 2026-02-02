import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
import { setupGUI } from '../mujocoUtils.js';
import {
  ensureMujocoReady,
  resolveSceneFileFromTask,
  loadSceneWithFallback,
} from './sceneResolver.js';

async function initDemo(demo, initialScene) {
  try {
    const mujoco = await ensureMujocoReady(demo, initialScene);
    const sceneFileToLoad = await resolveSceneFileFromTask(demo, mujoco, initialScene);
    await loadSceneWithFallback(demo, mujoco, sceneFileToLoad, initialScene);
    // Ensure name buffer is ready for actuator/joint lookups (used by replay conversion)
    demo.refreshNameBuffer();
    demo.setupRobotConfig();
    demo.setRobotReadyPose();
    demo.gui = new GUI();
    setupGUI(demo);
    try {
      demo.setupIKController();
    } catch (error) {
      console.error('[init] Failed to setup IKController:', error);
      demo.ikController = null;
    }
    if (demo.initialStateConfig) {
      console.log('[InitialState] Applying task initial_state...');
      demo.applyInitialState(demo.initialStateConfig);
    }
    demo.captureInitialState();
    if (demo.domainRandomizationConfig) {
      demo.applyDomainRandomization(demo.domainRandomizationConfig);
      demo.mujoco.mj_forward(demo.model, demo.data);
      if (demo.ikController) {
        demo.ikController.syncCtrlFromQpos();
        demo.ikController.syncTargetsFromModel();
      }
    }
    demo.clearEpisodeState();
    demo.updateSceneFromData();

    // Start status broadcasting for React integration
    demo.startStatusBroadcasting();
    demo.emitLoadingProgress({
      phase: "ready",
      message: "Ready",
      percent: 100,
    });
  } catch (error) {
    console.error('[init] Initialization error:', error);
    demo.emitLoadingProgress({
      phase: "error",
      message: error?.message ? `Init failed: ${error.message}` : "Init failed",
    });
    throw error;
  }
}

function setupReplay(demo) {
  // Check for pending replay samples (from React parent or window.parent for iframe compatibility)
  let samples = null;
  if (demo.pendingReplaySamples && demo.pendingReplayFlag) {
    samples = demo.pendingReplaySamples;
    console.log(`[setupReplay] Found pending replay samples in instance: ${samples.length} samples`);
  } else if (window.parent && window.parent.__pendingReplayFlag && window.parent.__pendingReplaySamples) {
    samples = window.parent.__pendingReplaySamples;
    console.log(`[setupReplay] Found pending replay samples in parent window: ${samples.length} samples`);
    // Clear the parent window flags
    window.parent.__pendingReplayFlag = false;
    window.parent.__pendingReplaySamples = null;
  }

  if (samples && Array.isArray(samples) && samples.length > 0) {
    // Store in this instance
    demo.pendingReplaySamples = samples;
    demo.pendingReplayFlag = true;
    // Load and start replay
    demo.loadTrajectoryFromData(samples);
  } else {
    console.log(`[setupReplay] No pending replay samples found`);
  }
}

function onModelReloaded(demo) {
  // IMPORTANT: refresh name buffer before any logic that reads body/joint/actuator names.
  // setupIKController/setupGripperControls/setupCheckers all depend on readNameAt().
  demo.refreshNameBuffer();
  demo.setupRobotConfig();
  try {
    demo.setupIKController();
  } catch (error) {
    console.error('[onModelReloaded] Failed to setup IKController:', error);
    demo.ikController = null;
  }
  demo.setupGripperControls();
  demo.setupCheckers();
  demo.gripperClosed = false;
  demo.checkpointState = null;
  demo.checkpointSavedAt = null;
  demo.setRobotReadyPose();
  if (demo.initialStateConfig) {
    console.log('[InitialState] Applying task initial_state (onModelReloaded)...');
    demo.applyInitialState(demo.initialStateConfig);
  }
  demo.captureInitialState();
  demo.clearEpisodeState();
  demo.params.paused = false;
  demo.guiPauseController?.setValue(false);
  demo.resetSuccessTimer();
  demo.resetSimulation();

  // Setup replay if pending samples exist
  demo.setupReplay();
}

export { initDemo, setupReplay, onModelReloaded };
