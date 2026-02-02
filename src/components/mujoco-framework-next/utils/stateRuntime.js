import * as THREE from 'three';

function parseVec3(value) {
  if (Array.isArray(value) && value.length >= 3) {
    return { x: Number(value[0]), y: Number(value[1]), z: Number(value[2]) };
  }
  return null;
}

function parseQuatWXYZ(value) {
  if (Array.isArray(value) && value.length >= 4) {
    return { w: Number(value[0]), x: Number(value[1]), y: Number(value[2]), z: Number(value[3]) };
  }
  return null;
}

function parseRange(value) {
  if (Array.isArray(value) && value.length === 2) {
    const [min, max] = value.map(Number);
    if (Number.isFinite(min) && Number.isFinite(max)) return [min, max];
  }
  if (value && typeof value === "object") {
    const min = Number(value.min);
    const max = Number(value.max);
    if (Number.isFinite(min) && Number.isFinite(max)) return [min, max];
  }
  if (Number.isFinite(Number(value))) {
    const v = Number(value);
    return [v, v];
  }
  return null;
}

function parseVec3Ranges(value) {
  if (!Array.isArray(value) || value.length !== 3) return null;
  const ranges = value.map(parseRange);
  if (ranges.some((r) => !r)) return null;
  return ranges;
}

function randomInRange(range) {
  const [min, max] = range;
  return min + Math.random() * (max - min);
}

function resolveJointNameCandidates(entityKey, entityCfg, jointKey) {
  const candidates = [];
  if (jointKey && jointKey.includes('/')) {
    candidates.push(jointKey);
    return candidates;
  }
  const mjcf = entityCfg?.mjcf || {};
  const jointPrefix = mjcf.jointPrefix || mjcf.joint_prefix || null;

  if (entityKey === 'franka') {
    candidates.push(`franka/${jointKey}`);
  }
  if (jointPrefix) {
    candidates.push(`${jointPrefix}/${jointKey}`);
  }
  candidates.push(`${entityKey}/${jointKey}`);
  candidates.push(`${entityKey}_base/${jointKey}`);
  candidates.push(`${entityKey}_base/${entityKey}_${jointKey}`);
  candidates.push(jointKey);
  return Array.from(new Set(candidates));
}

function trySetJointScalarQpos(demo, jointName, value, isDelta = false) {
  if (!jointName) return false;
  const jointId = demo.findIdByName(jointName, demo.model.name_jntadr, demo.model.njnt);
  if (jointId < 0) return false;
  const addr = demo.model.jnt_qposadr[jointId];
  if (addr < 0 || addr >= demo.data.qpos.length) return false;
  const target = isDelta ? demo.data.qpos[addr] + value : value;
  demo.data.qpos[addr] = target;

  const directActuatorIdx = demo.findIdByName(jointName, demo.model.name_actuatoradr, demo.model.nu);
  if (directActuatorIdx >= 0 && directActuatorIdx < demo.data.ctrl.length) {
    demo.data.ctrl[directActuatorIdx] = target;
    return true;
  }
  for (let a = 0; a < demo.model.nu; a++) {
    const actuatorName = demo.readNameAt(demo.model.name_actuatoradr[a]);
    if (actuatorName === jointName || actuatorName.endsWith('/' + jointName) || actuatorName.endsWith(jointName)) {
      if (a < demo.data.ctrl.length) demo.data.ctrl[a] = target;
      break;
    }
  }
  return true;
}

function trySetBodyFreeJointPose(demo, entityKey, entityCfg) {
  const pos = parseVec3(entityCfg?.pos);
  const quat = parseQuatWXYZ(entityCfg?.rot);
  if (!pos || !quat) return false;

  const mjcf = entityCfg?.mjcf || {};
  const bodyNameCandidates = [];
  if (mjcf.body) bodyNameCandidates.push(mjcf.body);
  if (mjcf.bodyName) bodyNameCandidates.push(mjcf.bodyName);
  if (entityKey === 'franka') bodyNameCandidates.push('franka/');
  bodyNameCandidates.push(`${entityKey}/`);
  bodyNameCandidates.push(`${entityKey}_base/`);
  bodyNameCandidates.push(entityKey);

  let bodyId = -1;
  for (const bn of bodyNameCandidates) {
    const id = demo.findIdByName(bn, demo.model.name_bodyadr, demo.model.nbody);
    if (id >= 0) { bodyId = id; break; }
  }
  if (bodyId < 0) return false;

  const mjJNT_FREE = demo.mujoco.mjtJoint.mjJNT_FREE.value;
  let freeJointId = -1;
  for (let j = 0; j < demo.model.njnt; j++) {
    if (demo.model.jnt_bodyid[j] === bodyId && demo.model.jnt_type[j] === mjJNT_FREE) {
      freeJointId = j;
      break;
    }
  }
  if (freeJointId < 0) {
    console.warn(`[InitialState] Body "${demo.readNameAt(demo.model.name_bodyadr[bodyId])}" has no FREE joint; cannot apply pos/rot at runtime.`);
    return false;
  }

  const addr = demo.model.jnt_qposadr[freeJointId];
  if (addr < 0 || (addr + 6) >= demo.data.qpos.length) return false;
  // MuJoCo free joint qpos layout: [x y z qw qx qy qz]
  demo.data.qpos[addr + 0] = pos.x;
  demo.data.qpos[addr + 1] = pos.y;
  demo.data.qpos[addr + 2] = pos.z;
  demo.data.qpos[addr + 3] = quat.w;
  demo.data.qpos[addr + 4] = quat.x;
  demo.data.qpos[addr + 5] = quat.y;
  demo.data.qpos[addr + 6] = quat.z;
  return true;
}

function tryRandomizeBodyFreeJoint(demo, entityKey, entityCfg, posRanges, posIsDelta, rotRanges, rotIsDelta) {
  if (!posRanges && !rotRanges) return false;
  const mjcf = entityCfg?.mjcf || {};
  const bodyNameCandidates = [];
  if (mjcf.body) bodyNameCandidates.push(mjcf.body);
  if (mjcf.bodyName) bodyNameCandidates.push(mjcf.bodyName);
  if (entityKey === 'franka') bodyNameCandidates.push('franka/');
  bodyNameCandidates.push(`${entityKey}/`);
  bodyNameCandidates.push(`${entityKey}_base/`);
  bodyNameCandidates.push(entityKey);

  let bodyId = -1;
  for (const bn of bodyNameCandidates) {
    const id = demo.findIdByName(bn, demo.model.name_bodyadr, demo.model.nbody);
    if (id >= 0) { bodyId = id; break; }
  }
  if (bodyId < 0) return false;

  const mjJNT_FREE = demo.mujoco.mjtJoint.mjJNT_FREE.value;
  let freeJointId = -1;
  for (let j = 0; j < demo.model.njnt; j++) {
    if (demo.model.jnt_bodyid[j] === bodyId && demo.model.jnt_type[j] === mjJNT_FREE) {
      freeJointId = j;
      break;
    }
  }
  if (freeJointId < 0) {
    console.warn(`[DomainRandomization] Body "${demo.readNameAt(demo.model.name_bodyadr[bodyId])}" has no FREE joint; cannot randomize pos.`);
    return false;
  }

  const addr = demo.model.jnt_qposadr[freeJointId];
  if (addr < 0 || (addr + 6) >= demo.data.qpos.length) return false;
  if (posRanges) {
    const current = {
      x: demo.data.qpos[addr + 0],
      y: demo.data.qpos[addr + 1],
      z: demo.data.qpos[addr + 2],
    };
    const newPos = {
      x: posIsDelta ? current.x + randomInRange(posRanges[0]) : randomInRange(posRanges[0]),
      y: posIsDelta ? current.y + randomInRange(posRanges[1]) : randomInRange(posRanges[1]),
      z: posIsDelta ? current.z + randomInRange(posRanges[2]) : randomInRange(posRanges[2]),
    };
    demo.data.qpos[addr + 0] = newPos.x;
    demo.data.qpos[addr + 1] = newPos.y;
    demo.data.qpos[addr + 2] = newPos.z;
  }
  if (rotRanges) {
    const currentQuat = new THREE.Quaternion(
      demo.data.qpos[addr + 4],
      demo.data.qpos[addr + 5],
      demo.data.qpos[addr + 6],
      demo.data.qpos[addr + 3]
    );
    let nextQuat = currentQuat;
    const deltaEuler = new THREE.Euler(
      randomInRange(rotRanges[0]),
      randomInRange(rotRanges[1]),
      randomInRange(rotRanges[2]),
      "XYZ"
    );
    const deltaQuat = new THREE.Quaternion().setFromEuler(deltaEuler);
    if (rotIsDelta) {
      nextQuat = currentQuat.clone().multiply(deltaQuat).normalize();
    } else {
      nextQuat = deltaQuat.normalize();
    }
    demo.data.qpos[addr + 3] = nextQuat.w;
    demo.data.qpos[addr + 4] = nextQuat.x;
    demo.data.qpos[addr + 5] = nextQuat.y;
    demo.data.qpos[addr + 6] = nextQuat.z;
  }
  return true;
}

function applyInitialState(demo, initialState) {
  if (!initialState || !demo.mujoco || !demo.model || !demo.data) return;

  const trySetJoint = (jointName, value) => {
    const jointId = demo.findIdByName(jointName, demo.model.name_jntadr, demo.model.njnt);
    if (jointId < 0) return false;
    const jointType = demo.model.jnt_type[jointId];
    if (jointType === demo.mujoco.mjtJoint.mjJNT_FREE.value) return false;
    return trySetJointScalarQpos(demo, jointName, value, false);
  };

  const applyEntity = (entityKey, entityCfg) => {
    if (!entityCfg) return;

    // Pose (only if movable via FREE joint)
    trySetBodyFreeJointPose(demo, entityKey, entityCfg);

    // Joint dof_pos
    const dofPos = entityCfg.dof_pos || entityCfg.dofPos || null;
    if (dofPos && typeof dofPos === 'object') {
      for (const [jointKey, value] of Object.entries(dofPos)) {
        const v = Number(value);
        if (!Number.isFinite(v)) continue;
        const candidates = resolveJointNameCandidates(entityKey, entityCfg, jointKey);
        let applied = false;
        for (const name of candidates) {
          if (trySetJoint(name, v)) { applied = true; break; }
        }
        if (!applied) {
          console.warn(`[InitialState] Could not resolve joint "${jointKey}" for "${entityKey}". Tried: ${candidates.join(', ')}`);
        }
      }
    }
  };

  try {
    const objects = initialState.objects || {};
    const robots = initialState.robots || {};

    for (const [k, cfg] of Object.entries(objects)) applyEntity(k, cfg);
    for (const [k, cfg] of Object.entries(robots)) applyEntity(k, cfg);

    demo.mujoco.mj_forward(demo.model, demo.data);
    demo.updateSceneFromData();

    if (demo.ikController) {
      demo.ikController.syncCtrlFromQpos();
      demo.ikController.syncTargetsFromModel();
    }
  } catch (e) {
    console.warn('[InitialState] Failed to apply initial_state:', e);
  }
}

function applyDomainRandomization(demo, domainRandomization) {
  const cfg = domainRandomization || demo.domainRandomizationConfig;
  if (!cfg || !demo.mujoco || !demo.model || !demo.data) { return; }

  const applyJointMap = (entityKey, entityCfg, jointMap, isDelta) => {
    if (!jointMap || typeof jointMap !== "object") return;
    for (const [jointKey, rawRange] of Object.entries(jointMap)) {
      const range = parseRange(rawRange);
      if (!range) continue;
      const value = randomInRange(range);
      const candidates = resolveJointNameCandidates(entityKey, entityCfg, jointKey);
      let applied = false;
      for (const name of candidates) {
        if (trySetJointScalarQpos(demo, name, value, isDelta)) { applied = true; break; }
      }
      if (!applied) {
        console.warn(`[DomainRandomization] Could not resolve joint "${jointKey}" for "${entityKey}". Tried: ${candidates.join(', ')}`);
      }
    }
  };

  const applyEntity = (entityKey, entityCfg) => {
    if (!entityCfg) return;
    const posDelta = entityCfg.pos_delta || entityCfg.posDelta || null;
    const posAbs = entityCfg.pos || null;
    const posRanges = parseVec3Ranges(posDelta || posAbs);
    const rotDelta = entityCfg.rot_delta || entityCfg.rotDelta || null;
    const rotAbs = entityCfg.rot || null;
    const rotRanges = parseVec3Ranges(rotDelta || rotAbs);
    if (posRanges || rotRanges) {
      tryRandomizeBodyFreeJoint(demo, entityKey, entityCfg, posRanges, !!posDelta, rotRanges, !!rotDelta);
    }

    const dofPos = entityCfg.dof_pos || entityCfg.dofPos || null;
    const dofPosDelta = entityCfg.dof_pos_delta || entityCfg.dofPosDelta || null;
    applyJointMap(entityKey, entityCfg, dofPos, false);
    applyJointMap(entityKey, entityCfg, dofPosDelta, true);
  };

  const objects = cfg.objects || {};
  const robots = cfg.robots || {};
  for (const [k, v] of Object.entries(objects)) applyEntity(k, v);
  for (const [k, v] of Object.entries(robots)) applyEntity(k, v);
}

export {
  applyInitialState,
  applyDomainRandomization,
};
