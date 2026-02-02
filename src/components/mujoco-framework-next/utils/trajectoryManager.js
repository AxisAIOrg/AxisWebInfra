function getTrajectoryFetchURL(path) {
  if (!path) { return null; }
  if (path.startsWith('http://') || path.startsWith('https://') || path.startsWith('/')) {
    return path;
  }
  return '/' + path;
}

async function startReplayTrajectory(demo) {
  if (!demo.lastSavedTrajectoryPath) { return; }
  const url = getTrajectoryFetchURL(demo.lastSavedTrajectoryPath);
  if (!url) { return; }
  try {
    const response = await fetch(url, { cache: 'no-store' });
    if (!response.ok) { return; }
    const payload = await response.json();
    if (!Array.isArray(payload.samples) || !payload.samples.length) { return; }
    demo.loadTrajectoryFromData(payload.samples);
  } catch (error) {
    console.error('[Teleop] Failed to load trajectory:', error);
  }
}

function loadTrajectoryFromData(demo, samples) {
  if (!Array.isArray(samples) || !samples.length) {
    console.error('[Teleop] Invalid trajectory samples:', samples);
    return;
  }
  if (!demo.model || !demo.data) {
    console.error('[Teleop] Model or data not initialized, cannot load trajectory');
    return;
  }

  console.log(`[Teleop] Converting and loading trajectory with ${samples.length} samples for replay`);
  console.log(`[Teleop] First sample structure:`, samples[0]);

  try {
    // Convert samples from API format (with joint names) to replay format (with qpos/qvel arrays)
    const convertedSamples = samples.map((sample, idx) => {
      const converted = {
        elapsedMs: sample.elapsedMs || 0,
        step: sample.step || idx,
        timestamp: sample.timestamp,
      };

      // Convert robot_joints object to qpos array
      // Always initialize qpos from current state to preserve non-robot joints
      const qpos = Array.from(demo.data.qpos);

      if (sample.robot_joints && typeof sample.robot_joints === 'object') {
        let mappedCount = 0;

        // Map each joint name to its qpos address and set the value
        for (const [jointName, value] of Object.entries(sample.robot_joints)) {
          const address = demo.getJointAddress(jointName);
          if (address >= 0 && address < qpos.length) {
            qpos[address] = value;
            mappedCount++;
          } else {
            console.warn(`[Teleop] Joint ${jointName} not found or invalid address ${address} (qpos length: ${qpos.length})`);
          }
        }
        converted.qpos = qpos;
        if (idx === 0) {
          console.log(`[Teleop] Mapped ${mappedCount} robot joints to qpos array (total qpos: ${qpos.length})`);
        }
      } else if (sample.qpos && Array.isArray(sample.qpos)) {
        // Already in array format - use it directly but validate length
        if (sample.qpos.length === qpos.length) {
          converted.qpos = sample.qpos;
        } else {
          // Length mismatch - copy what we can
          console.warn(`[Teleop] Sample ${idx} qpos length mismatch: ${sample.qpos.length} vs ${qpos.length}, using provided array`);
          converted.qpos = sample.qpos;
        }
        if (idx === 0) {
          console.log(`[Teleop] Using direct qpos array format (length: ${sample.qpos.length})`);
        }
      } else {
        // No robot_joints or qpos - use current state (will replay current pose)
        converted.qpos = qpos;
        if (idx === 0) {
          console.warn(`[Teleop] Sample ${idx} has no qpos or robot_joints data, using current state`);
        }
      }

      // Convert robot_velocities object to qvel array
      if (sample.robot_velocities && typeof sample.robot_velocities === 'object') {
        // Initialize qvel array with current state
        const qvel = demo.data.qvel ? Array.from(demo.data.qvel) : [];

        // Map each joint name to its qvel address
        for (const [jointName, value] of Object.entries(sample.robot_velocities)) {
          const jointId = demo.findIdByName(jointName, demo.model.name_jntadr, demo.model.njnt);
          if (jointId >= 0) {
            const qvelAddress = demo.model.jnt_dofadr[jointId];
            if (qvelAddress >= 0 && qvelAddress < qvel.length) {
              // Handle both scalar and array values
              if (Array.isArray(value)) {
                for (let i = 0; i < value.length && (qvelAddress + i) < qvel.length; i++) {
                  qvel[qvelAddress + i] = value[i];
                }
              } else {
                qvel[qvelAddress] = value;
              }
            }
          }
        }
        converted.qvel = qvel;
      } else if (sample.qvel) {
        // Already in array format
        converted.qvel = sample.qvel;
      }

      // Convert controls object to ctrl array
      if (sample.controls && typeof sample.controls === 'object') {
        // Ensure we have name buffer for actuator lookup
        if (!demo.namesBuffer) {
          demo.refreshNameBuffer();
        }
        const ctrl = demo.data.ctrl ? Array.from(demo.data.ctrl) : [];

        // Map actuator names to ctrl indices
        for (let i = 0; i < demo.model.nu; i++) {
          const actuatorName = demo.readNameAt(demo.model.name_actuatoradr[i]);
          if (sample.controls.hasOwnProperty(actuatorName)) {
            ctrl[i] = sample.controls[actuatorName];
          }
        }
        converted.ctrl = ctrl;
      } else if (sample.ctrl) {
        // Already in array format
        converted.ctrl = sample.ctrl;
      }

      return converted;
    });

    console.log(`[Teleop] Conversion complete, ${convertedSamples.length} samples converted`);

    // Use EXACT same reset sequence as the Reset button (from mujoco-dashboard-framework.html)
    // This ensures all assets are loaded and simulation is in the exact same state
    console.log('[Teleop] Resetting simulation using exact Reset button logic...');

    // 1. Reset episode state
    demo.clearEpisodeState();

    // 2. Reset success flags
    demo.successAchieved = false;
    demo.successTime = null;
    demo.resetSuccessTimer();

    // 3. Reset simulation (this does restoreInitialState, mj_forward, updateSceneFromData, IK sync)
    demo.resetSimulation();

    // 4. Unpause
    demo.params.paused = false;
    demo.guiPauseController?.setValue(false);

    console.log('[Teleop] Reset complete, setting up replay...');

    // Validate converted samples
    const validSamples = convertedSamples.filter(s => s.qpos && s.qpos.length > 0);
    if (validSamples.length === 0) {
      console.error('[Teleop] No valid samples after conversion! All samples missing qpos data.');
      console.error('[Teleop] Sample check:', convertedSamples.slice(0, 3).map(s => ({ hasQpos: !!s.qpos, qposLength: s.qpos?.length })));
      return;
    }

    if (validSamples.length < convertedSamples.length) {
      console.warn(`[Teleop] Only ${validSamples.length} of ${convertedSamples.length} samples are valid`);
    }

    demo.replaySamples = validSamples;
    demo.replayIndex = 0;
    demo.replayStartTime = null;
    demo.replayPlaying = true;
    demo.replayLoopLogged = false; // log once when render enters replay loop

    // Reset sequence above already unpaused the simulation
    // The replay loop will naturally apply samples starting from index 0

    console.log(`[Teleop] Successfully loaded ${validSamples.length} trajectory samples for replay`);
    console.log(`[Teleop] Replay state: playing=${demo.replayPlaying}, samples=${demo.replaySamples.length}, first elapsedMs=${demo.replaySamples[0]?.elapsedMs}`);
    console.log(`[Teleop] Simulation paused=${demo.params.paused}, model=${!!demo.model}, data=${!!demo.data}`);
  } catch (error) {
    console.error('[Teleop] Error during trajectory conversion:', error);
    console.error('[Teleop] Error stack:', error.stack);
    demo.replayPlaying = false;
    demo.replaySamples = null;
  }
}

function stopReplayTrajectory(demo) {
  demo.replayPlaying = false;
  demo.replaySamples = null;
  demo.replayIndex = 0;
  demo.replayStartTime = null;
}

function updateReplay(demo, timeMS) {
  if (!demo.replayPlaying || !demo.replaySamples?.length) {
    if (!demo.replayPlaying) {
      console.warn('[Teleop] updateReplay called but replayPlaying is false');
    }
    if (!demo.replaySamples?.length) {
      console.warn('[Teleop] updateReplay called but no replay samples');
    }
    return;
  }
  if (demo.replayStartTime === null) {
    demo.replayStartTime = timeMS;
    console.log(`[Teleop] Replay started at timeMS=${timeMS}`);
  }
  const elapsed = timeMS - demo.replayStartTime;
  let appliedCount = 0;
  while (demo.replayIndex < demo.replaySamples.length &&
         demo.replaySamples[demo.replayIndex].elapsedMs <= elapsed) {
    demo.applyTrajectorySample(demo.replaySamples[demo.replayIndex]);
    demo.replayIndex++;
    appliedCount++;
  }
  if (appliedCount > 0 && demo.replayIndex % 10 === 0) {
    console.log(`[Teleop] Replay progress: ${demo.replayIndex}/${demo.replaySamples.length} samples (elapsed: ${elapsed.toFixed(1)}ms)`);
  }
  if (demo.replayIndex >= demo.replaySamples.length) {
    console.log('[Teleop] Replay completed!');
    demo.stopReplayTrajectory();
  }
}

function applyTrajectorySample(demo, sample) {
  if (!sample || !demo.data) {
    console.warn('[Teleop] applyTrajectorySample: missing sample or data', { hasSample: !!sample, hasData: !!demo.data });
    return;
  }
  let applied = false;
  if (sample.qpos && demo.data.qpos.length >= sample.qpos.length) {
    demo.data.qpos.set(sample.qpos);
    applied = true;
  }
  if (sample.qvel && demo.data.qvel?.length >= sample.qvel.length) {
    demo.data.qvel.set(sample.qvel);
  }
  if (sample.ctrl && demo.data.ctrl?.length >= sample.ctrl.length) {
    demo.data.ctrl.set(sample.ctrl);
  }
  if (sample.mocap_pos && demo.model.nmocap > 0) {
    demo.data.mocap_pos.set(sample.mocap_pos);
  }
  if (sample.mocap_quat && demo.model.nmocap > 0) {
    demo.data.mocap_quat.set(sample.mocap_quat);
  }
  if (applied) {
    demo.mujoco.mj_forward(demo.model, demo.data);
    demo.updateSceneFromData();
  } else {
    console.warn('[Teleop] applyTrajectorySample: no qpos applied', {
      hasQpos: !!sample.qpos,
      qposLength: sample.qpos?.length,
      dataQposLength: demo.data.qpos.length
    });
  }
}

function recordEpisodeSample(demo, timeMS, force = false) {
  if (demo.episodeStartTime === null) {
    demo.episodeStartTime = timeMS;
    demo.episodeStartWallClock = Date.now();
    demo.lastEpisodeSample = timeMS - demo.episodeSampleIntervalMs;
    demo.episodeStepCount = 0;
  }
  let didStep = false;
  if (!force) {
    if (timeMS - demo.lastEpisodeSample < demo.episodeSampleIntervalMs) {
      return { didStep: false, didRecord: false };
    }
    demo.episodeStepCount++;
    didStep = true;
    if (demo.episodeStepCount % demo.decimation !== 0) {
      return { didStep, didRecord: false };
    }
  }
  demo.lastEpisodeSample = timeMS;
  const sample = {
    timestamp: timeMS,
    elapsedMs: timeMS - demo.episodeStartTime,
    step: demo.episodeStepCount,
    qpos: Array.from(demo.data.qpos),
    qvel: Array.from(demo.data.qvel),
  };

  if (demo.data.ctrl?.length) {
    sample.ctrl = Array.from(demo.data.ctrl);
  }

  if (demo.model.nmocap > 0) {
    sample.mocap_pos = Array.from(demo.data.mocap_pos);
    sample.mocap_quat = Array.from(demo.data.mocap_quat);
  }

  // Save body states for object position/orientation extraction
  if (demo.data.xpos && demo.model.nbody > 0) {
    sample.xpos = Array.from(demo.data.xpos);
  }
  if (demo.data.xquat && demo.model.nbody > 0) {
    sample.xquat = Array.from(demo.data.xquat);
  }
  if (demo.data.xvel && demo.model.nbody > 0) {
    sample.xvel = Array.from(demo.data.xvel);
  }
  if (demo.data.cvel && demo.model.nbody > 0) {
    sample.cvel = Array.from(demo.data.cvel);
  }

  // Save keyboard state if available
  if (demo.keyboardStateManager) {
    // Get currently active keys (keys that are being held down)
    const activeKeys = demo.keyboardStateManager.activeKeys;
    if (activeKeys && activeKeys.size > 0) {
      sample.keyboard = Array.from(activeKeys);
    } else {
      // If no active keys, use empty array instead of null
      sample.keyboard = [];
    }
  }

  // Save mouse state if available (currently dragStateManager is disabled, so this will be null)
  // This is a placeholder for future mouse interaction recording
  sample.mouse = null;

  demo.currentEpisode.push(sample);
  return { didStep, didRecord: true };
}

async function saveTrajectory(payload) {
  if (typeof fetch !== 'function') {
    throw new Error('Fetch API unavailable in this environment.');
  }

  const response = await fetch('/api/trajectories', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload),
  });

  if (!response.ok) {
    let message = `HTTP ${response.status}`;
    try {
      const text = await response.text();
      if (text) { message += `: ${text}`; }
    } catch (_) {}
    throw new Error(message);
  }

  try {
    return await response.json();
  } catch (_) {
    return null;
  }
}

function serializeTrajectory(demo) {
  if (!demo.currentEpisode || demo.currentEpisode.length === 0) {
    return [];
  }

  const robotJoints = demo.robotControlConfig?.jointNames || [];
  const gripperJoints = demo.robotConfig?.gripper?.fingerJointNames || demo.gripperFingerJointNames || [];
  const allJoints = [...robotJoints, ...gripperJoints];
  const jointAddresses = {};
  const jointVelAddresses = {};

  // Cache joint addresses for positions
  allJoints.forEach(name => {
    const addr = demo.getJointAddress(name);
    if (addr >= 0) jointAddresses[name] = addr;
    
    // Cache joint velocity addresses
    const jointId = demo.findIdByName(name, demo.model.name_jntadr, demo.model.njnt);
    if (jointId >= 0) {
      const qvelAddr = demo.model.jnt_dofadr[jointId];
      if (qvelAddr >= 0) jointVelAddresses[name] = qvelAddr;
    }
  });

  // Get episode start time (Unix timestamp in milliseconds)
  const episodeStartUnix = demo.episodeStartWallClock || Date.now();

  // Helper function to get position from xpos array
  const getPos = (xpos, bodyId) => {
    if (!xpos || bodyId < 0 || bodyId * 3 + 2 >= xpos.length) return null;
    return [xpos[bodyId * 3], xpos[bodyId * 3 + 1], xpos[bodyId * 3 + 2]];
  };

  // Helper function to get quaternion from xquat array
  const getQuat = (xquat, bodyId) => {
    if (!xquat || bodyId < 0 || bodyId * 4 + 3 >= xquat.length) return null;
    return [xquat[bodyId * 4], xquat[bodyId * 4 + 1], xquat[bodyId * 4 + 2], xquat[bodyId * 4 + 3]];
  };

  return demo.currentEpisode.map(sample => {
    // Map robot joints (including gripper joints)
    const currentRobotJoints = {};
    allJoints.forEach(name => {
      const addr = jointAddresses[name];
      if (addr !== undefined && sample.qpos && addr < sample.qpos.length) {
        currentRobotJoints[name] = sample.qpos[addr];
      }
    });

    // Map robot velocities (including gripper joints)
    const currentRobotVelocities = {};
    allJoints.forEach(name => {
      const qvelAddr = jointVelAddresses[name];
      if (qvelAddr !== undefined && sample.qvel && qvelAddr < sample.qvel.length) {
        // For most joints, velocity is a scalar
        currentRobotVelocities[name] = sample.qvel[qvelAddr];
      }
    });

    // Extract object positions, orientations, and velocities
    const objectPositions = {};
    const objectOrientations = {};
    const objectVelocities = {};
    const objectAngularVelocities = {};

    if (sample.xpos && sample.xquat && demo.model) {
      // Extract all bodies except robot parts
      for (let b = 0; b < demo.model.nbody; b++) {
        try {
          const bodyName = demo.readNameAt(demo.model.name_bodyadr[b]);
          
          // Skip if name is null, empty, or robot-related
          if (!bodyName || bodyName.trim() === '' || 
              bodyName.startsWith('franka/') || 
              bodyName.startsWith('base_link') ||
              bodyName.startsWith('panda_')) {
            continue;
          }

          // Extract position
          const pos = getPos(sample.xpos, b);
          if (pos) {
            objectPositions[bodyName] = pos;
          }

          // Extract orientation
          const quat = getQuat(sample.xquat, b);
          if (quat) {
            objectOrientations[bodyName] = quat;
          }

          // Extract velocities if available
          if (sample.cvel && b * 6 + 5 < sample.cvel.length) {
            // cvel contains [lin_vel_x, lin_vel_y, lin_vel_z, ang_vel_x, ang_vel_y, ang_vel_z] per body
            objectVelocities[bodyName] = [
              sample.cvel[b * 6],
              sample.cvel[b * 6 + 1],
              sample.cvel[b * 6 + 2]
            ];
            objectAngularVelocities[bodyName] = [
              sample.cvel[b * 6 + 3],
              sample.cvel[b * 6 + 4],
              sample.cvel[b * 6 + 5]
            ];
          } else if (sample.xvel && b * 3 + 2 < sample.xvel.length) {
            // xvel contains linear velocity only
            objectVelocities[bodyName] = [
              sample.xvel[b * 3],
              sample.xvel[b * 3 + 1],
              sample.xvel[b * 3 + 2]
            ];
            // Angular velocity not available in xvel
            objectAngularVelocities[bodyName] = [0.0, 0.0, 0.0];
          }
        } catch (e) {
          // Skip if error reading body name or extracting data
          continue;
        }
      }
    }

    // Extract controls if available
    const controls = {};
    if (sample.ctrl && demo.model) {
      for (let i = 0; i < demo.model.nu; i++) {
        try {
          const actuatorName = demo.readNameAt(demo.model.name_actuatoradr[i]);
          if (actuatorName && i < sample.ctrl.length) {
            controls[actuatorName] = sample.ctrl[i];
          }
        } catch (e) {
          // Skip if error reading actuator name
          continue;
        }
      }
    }

    // Extract keyboard state if available
    // Use empty array if no keys are pressed, null only if keyboardStateManager doesn't exist
    const keyboard = sample.keyboard !== undefined ? sample.keyboard : null;

    // Extract mouse state if available (currently null as dragStateManager is disabled)
    const mouse = sample.mouse !== undefined ? sample.mouse : null;

    // Convert to Unix timestamp: episodeStartUnix + elapsedMs, rounded to integer
    const unixTimestamp = Math.round(episodeStartUnix + sample.elapsedMs);

    return {
      timestamp: unixTimestamp, // Integer Unix timestamp in milliseconds
      simulation_time: sample.elapsedMs / 1000,
      state: {
        robot_joints: currentRobotJoints,
        robot_velocities: currentRobotVelocities,
        object_positions: objectPositions,
        object_velocities: objectVelocities,
        object_orientations: objectOrientations,
        object_angular_velocities: objectAngularVelocities
      },
      action: {
        keyboard: keyboard,
        mouse: mouse,
        controls: controls
      }
    };
  });
}

function buildEpisodeMetadata(demo, episode) {
  if (!episode || episode.length === 0) {
    return {
      sampleCount: 0,
      durationMs: 0,
      desiredIntervalMs: demo.episodeSampleIntervalMs || 50,
    };
  }

  const lastSample = episode[episode.length - 1];
  return {
    sampleCount: episode.length,
    durationMs: lastSample.elapsedMs || 0,
    desiredIntervalMs: demo.episodeSampleIntervalMs || 50,
  };
}

export {
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
};
