import { CheckerManager } from '../checkers/CheckerManager.js';

function setupCheckers(demo) {
  // Reset
  demo.checkerManager = null;

  if (!demo.checkerConfig) {
    console.log('[Teleop] No checker config provided from task, skipping checker setup');
    return;
  }

  console.log('[Teleop] Setting up checkers from task config:', JSON.stringify(demo.checkerConfig, null, 2));
  try {
    demo.checkerManager = new CheckerManager(
      demo.mujoco,
      demo.model,
      demo.data,
      demo.checkerConfig,
      { keyboardStateManager: demo.keyboardStateManager }
    );
    console.log('[Teleop] CheckerManager initialized from task config');
  } catch (error) {
    console.error('[Teleop] Failed to setup CheckerManager from task config:', error);
    demo.checkerManager = null;
  }
}

function updateCheckers(demo, timeMS) {
  if (timeMS - demo.lastCheckerUpdate < demo.checkerUpdateInterval) {
    return;
  }
  demo.lastCheckerUpdate = timeMS;
  if (!demo.checkerManager) return;
  try {
    // Poll status periodically (also useful for updating internal caches)
    demo.checkerManager.getStatus();
  } catch (error) {
    console.error('[Checker] CheckerManager error:', error);
  }
}

function getCheckerStatuses(demo) {
  if (!demo.checkerManager) return {};
  try {
    return demo.checkerManager.getStatus();
  } catch (error) {
    return { error: error.message };
  }
}

function maybeFinalizeSuccess(demo, timeMS) {
  // Export trajectory exactly once when success is detected.
  // This is the "finish checker" bridge: checker -> success -> pause -> export.
  if (demo.successExported) return;
  if (!demo.onTrajectoryExport) return;
  if (!demo.model || !demo.data) return;
  if (demo.replayPlaying) return;

  let checkerSuccessNow = false;
  try {
    if (demo.checkerManager) {
      checkerSuccessNow = !!demo.checkerManager.check();
    }
  } catch (e) {
    // ignore (status broadcasting will surface error)
  }

  if (!demo.successAchieved && !checkerSuccessNow) return;

  // Mark exported first to avoid any re-entrancy / repeated calls.
  demo.successExported = true;

  // Pause simulation so the UI time stops and state is stable.
  demo.params.paused = true;
  if (demo.guiPauseController) {
    demo.guiPauseController.setValue(true);
  }

  // Ensure we have at least one final sample at the moment of completion.
  try {
    demo.recordEpisodeSample(timeMS, true);
  } catch (e) {
    console.warn('[Teleop] Failed to record final episode sample on success:', e);
  }

  try {
    const serializedTrajectory = demo.serializeTrajectory();
    const metadata = demo.buildEpisodeMetadata(demo.currentEpisode);
    demo.onTrajectoryExport(serializedTrajectory, metadata);
  } catch (e) {
    console.error('[Teleop] Failed to export trajectory on success:', e);
  }

  // Push an immediate status update (will reflect paused + success).
  demo.broadcastStatus();
}

export {
  setupCheckers,
  updateCheckers,
  getCheckerStatuses,
  maybeFinalizeSuccess,
};
