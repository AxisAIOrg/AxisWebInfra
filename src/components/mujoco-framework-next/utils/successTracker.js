import * as THREE from 'three';

function updateSuccessTracking(demo) {
  // Check success conditions every step (not just when printing)
  if (!demo.successAchieved) {
    try {
      // Initialize success timer if not started
      if (demo.successStartTime === null) {
        demo.resetSuccessTimer();
      }

      // Prefer dynamic checker config from backend (CheckerManager)
      let isSuccessNow = false;
      if (demo.checkerManager) {
        isSuccessNow = !!demo.checkerManager.check();
      } else {
        // Legacy fallback (should be unused once tasks provide checker_config)
        let bowlInRange = false;
        if (demo.bowlPositionChecker) {
          const minBounds = new THREE.Vector3(0.3, 0.14, 0.12);
          const maxBounds = new THREE.Vector3(0.7, 0.22, 0.2);
          bowlInRange = demo.bowlPositionChecker.checkInBounds(minBounds, maxBounds);
        }
        let drawerOpen = false;
        if (demo.drawerPositionChecker) {
          const drawerStatus = demo.drawerPositionChecker.getStatus();
          drawerOpen = drawerStatus.success;
        }
        isSuccessNow = bowlInRange && drawerOpen;
      }

      if (isSuccessNow) {
        demo.successAchieved = true;
        demo.successTime = performance.now() - demo.successStartTime;
        const successTimeSeconds = (demo.successTime / 1000).toFixed(2);
        console.log(`🎉🎉🎉 SUCCESS ACHIEVED! Time: ${successTimeSeconds}s (${demo.successTime.toFixed(0)}ms) 🎉🎉🎉`);
      }
    } catch (error) {
      console.error(`[Step ${demo.episodeStepCount}] Failed to check success conditions:`, error);
    }
  }

  // Print per-subgoal progress for current task every N steps (only while not yet success).
  if (!demo.successAchieved && demo.checkerManager && demo.checkerProgressPrintEverySteps > 0) {
    if (demo.episodeStepCount % demo.checkerProgressPrintEverySteps === 0) {
      try {
        const report = demo.checkerManager.getProgressReport?.();
        if (report && Array.isArray(report.goals) && report.goals.length > 0) {
          const parts = report.goals.map(g => {
            const okMark = g.ok ? '✅' : '❌';
            const cur = typeof g.current === 'number' ? g.current : null;
            const thr = typeof g.threshold === 'number' ? g.threshold : null;
            const unit = g.unit === 'm' ? 'm' : (g.unit === 'deg' ? '°' : (g.unit || ''));
            if (cur !== null && thr !== null) {
              const curFmt = (g.unit === 'deg') ? cur.toFixed(1) : cur.toFixed(3);
              const thrFmt = (g.unit === 'deg') ? thr.toFixed(1) : thr.toFixed(3);
              return `${g.key}: ${curFmt}/${thrFmt}${unit} ${okMark}`;
            }
            return `${g.key}: ${okMark}`;
          });
          const line = `[Step ${demo.episodeStepCount}] Goals: ${parts.join(' | ')}`;
          // Avoid spamming identical lines (but still prints periodically when progress changes).
          if (line !== demo._lastCheckerProgressLine) {
            console.log(line);
            demo._lastCheckerProgressLine = line;
          }
        }
      } catch (e) {
        console.warn('[Checker] Failed to print progress report:', e);
      }
    }
  }

  // Print bowl position and drawer dof pos every 20 steps
  if (demo.episodeStepCount % 20 === 0) {
    try {
      // Print bowl position
      if (demo.bowlPositionChecker) {
        const bowlStatus = demo.bowlPositionChecker.getStatus();
        console.log(`[Step ${demo.episodeStepCount}] Bowl position: (${bowlStatus.position.x.toFixed(4)}, ${bowlStatus.position.y.toFixed(4)}, ${bowlStatus.position.z.toFixed(4)})`);

        // Check if bowl is in success range [0.3, 0.14, 0.12] to [0.7, 0.22, 0.2]
        const minBounds = new THREE.Vector3(0.3, 0.14, 0.12);
        const maxBounds = new THREE.Vector3(0.7, 0.22, 0.2);
        const bowlInRange = demo.bowlPositionChecker.checkInBounds(minBounds, maxBounds);
        if (bowlInRange) {
          console.log(`[Step ${demo.episodeStepCount}] ✅ Bowl is in target range!`);
        }
      }

      // Print drawer joint positions (dof pos)
      if (demo.drawerPositionChecker) {
        const drawerPositions = demo.drawerPositionChecker.getDrawerPositions();
        const drawerPosStr = Object.entries(drawerPositions)
          .map(([name, pos]) => `${name}: ${pos.toFixed(4)}`)
          .join(', ');
        console.log(`[Step ${demo.episodeStepCount}] Drawer dof pos: ${drawerPosStr}`);

        // Check if drawer is open (position < -0.1)
        const drawerStatus = demo.drawerPositionChecker.getStatus();
        if (drawerStatus.success) {
          console.log(`[Step ${demo.episodeStepCount}] ✅ Drawer is open! (position: ${drawerStatus.minPosition.toFixed(4)} < -0.1)`);
        }
      }
    } catch (error) {
      console.error(`[Step ${demo.episodeStepCount}] Failed to get positions:`, error);
    }
  }

  // Print box dof pos every 30 steps (task4 close_box)
  if (demo.episodeStepCount % 30 === 0) {
    try {
      const boxJointName = 'box_base/box_joint';
      const addr = demo.getJointAddress(boxJointName);
      if (addr >= 0 && addr < demo.data.qpos.length) {
        console.log(`[Step ${demo.episodeStepCount}] Box dof pos: ${boxJointName}: ${demo.data.qpos[addr].toFixed(4)}`);
      }
    } catch (error) {
      console.error(`[Step ${demo.episodeStepCount}] Failed to get box dof pos:`, error);
    }
  }
}

export { updateSuccessTracking };
