function broadcastStatus(demo) {
  if (!demo.onStatusUpdate) return;

  try {
    // Count completed episodes + current episode if it has samples
    const completedEpisodes = demo.episodes ? demo.episodes.length : 0;
    const currentEpisode = demo.currentEpisode || [];
    const currentEpisodeActive = currentEpisode.length > 0 ? 1 : 0;

    // Calculate current episode duration and sample count
    let currentDurationMs = 0;
    let currentSampleCount = currentEpisode.length;
    if (currentEpisode.length > 0) {
      const lastSample = currentEpisode[currentEpisode.length - 1];
      currentDurationMs = lastSample.elapsedMs || 0;
    }

    // Calculate simulation elapsed time from episode start
    let simulationElapsedMs = 0;
    if (demo.episodeStartTime !== null && demo.episodeStartTime !== undefined) {
      if (currentEpisode.length > 0) {
        simulationElapsedMs = currentDurationMs;
      } else {
        const currentTime = performance.now();
        simulationElapsedMs = currentTime - demo.episodeStartTime;
      }
    }

    // Get trajectory steps (current episode samples)
    const trajectorySteps = currentEpisode.map(sample => ({
      step: sample.step,
      timestamp: sample.timestamp,
      elapsedMs: sample.elapsedMs,
      qpos: sample.qpos ? sample.qpos.slice(0, 10) : [], // First 10 positions for preview
      qvel: sample.qvel ? sample.qvel.slice(0, 5) : [], // First 5 velocities
    }));

    let checkerStatus = null;
    let checkerSuccess = false;
    try {
      checkerStatus = demo.getCheckerStatuses();
      if (demo.checkerManager) {
        checkerSuccess = !!demo.checkerManager.check();
      }
    } catch (e) {
      checkerStatus = { error: e?.message || String(e) };
    }

    const status = {
      time: simulationElapsedMs,
      trajectoryCount: completedEpisodes + currentEpisodeActive,
      isSuccess: demo.successAchieved || checkerSuccess || false,
      currentTrajectory: {
        recording: currentEpisodeActive > 0,
        samples: currentSampleCount,
        durationMs: currentDurationMs,
        durationSec: (currentDurationMs / 1000).toFixed(1),
      },
      completedTrajectories: completedEpisodes,
      trajectorySteps: trajectorySteps,
      checkerStatus,
    };

    demo.onStatusUpdate(status);
  } catch (e) {
    console.error("[MuJoCoDemo] Error broadcasting status:", e);
  }
}

function startStatusBroadcasting(demo) {
  if (demo.statusBroadcastInterval) {
    clearInterval(demo.statusBroadcastInterval);
  }
  // Broadcast status every 100ms (10 updates per second)
  demo.statusBroadcastInterval = setInterval(() => {
    demo.broadcastStatus();
  }, 100);
}

function stopStatusBroadcasting(demo) {
  if (demo.statusBroadcastInterval) {
    clearInterval(demo.statusBroadcastInterval);
    demo.statusBroadcastInterval = null;
  }
}

export {
  broadcastStatus,
  startStatusBroadcasting,
  stopStatusBroadcasting,
};
