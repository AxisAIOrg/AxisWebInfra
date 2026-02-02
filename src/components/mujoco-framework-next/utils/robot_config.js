const FRANKA_READY_QPOS = new Float64Array([
  0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398, 0.04, 0.04,
]);

const FRANKA_JOINT_NAMES = [
  'franka/panda_joint1', 'franka/panda_joint2', 'franka/panda_joint3',
  'franka/panda_joint4', 'franka/panda_joint5', 'franka/panda_joint6',
  'franka/panda_joint7', 'franka/panda_finger_joint1', 'franka/panda_finger_joint2',
];

const DEFAULT_CONTROL_CONFIG = {
  endEffectorBodyName: null,
  jointNames: [],
  mode: 'joint',
  translationGain: 1.0,
  rotationGain: 0.5,
  maxIterations: 6,
  stepLimit: 0.04,
};

const ROBOT_CONFIGS = {
  franka: {
    key: 'franka',
    match: {
      includes: ['franka_panda_default.xml', 'franka_panda'],
      jointPrefix: 'franka/panda_joint',
    },
    controlConfig: {
      endEffectorBodyName: 'franka/panda_hand',
      jointNames: [
        'franka/panda_joint1',
        'franka/panda_joint2',
        'franka/panda_joint3',
        'franka/panda_joint4',
        'franka/panda_joint5',
        'franka/panda_joint6',
        'franka/panda_joint7',
      ],
      mode: 'joint',
      translationGain: 1.5,
      rotationGain: 0.8,
      maxIterations: 6,
      stepLimit: 0.04,
    },
    endEffectorCandidates: ['franka/panda_hand'],
    gripper: {
      fingerJointNames: ['franka/panda_finger_joint1', 'franka/panda_finger_joint2'],
      openPose: 0.04,
      closedPose: 0.0,
    },
    readyPose: {
      jointNames: FRANKA_JOINT_NAMES,
      qpos: FRANKA_READY_QPOS,
    },
    ik: {
      type: 'numeric',
    },
  },
  gen3_hand: {
    key: 'gen3_hand',
    match: {
      includes: ['gen3_hand.xml', 'includes/gen3_hand.xml', 'gen3.xml', 'gen3_with_shadow_hand'],
      jointNames: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'],
    },
    controlConfig: {
      endEffectorBodyName: 'lh_palm',
      jointNames: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'],
      mode: 'joint',
      translationGain: 1.5,
      rotationGain: 0.8,
      maxIterations: 6,
      stepLimit: 0.04,
    },
    endEffectorCandidates: ['lh_palm', 'lh_wrist', 'bracelet_link'],
    gripper: {
      fingerJointNames: [],
    },
    readyPose: {
      jointNames: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'],
      qpos: [0, 0.2618, 3.1416, -2.2689, 0, 0.9599, 1.5708],
    },
    ik: {
      type: 'numeric',
    },
  },
};

const DEFAULT_ROBOT_CONFIG = {
  key: 'unknown',
  controlConfig: { ...DEFAULT_CONTROL_CONFIG },
  endEffectorCandidates: [],
  gripper: { fingerJointNames: [] },
  readyPose: null,
  ik: { type: 'numeric' },
};

function extractIncludeFiles(xmlText) {
  if (!xmlText || typeof xmlText !== 'string') return [];
  const includes = [];
  const regex = /<include\s+file="([^"]+)"/g;
  let match = null;
  while ((match = regex.exec(xmlText)) !== null) {
    includes.push(match[1]);
  }
  return includes;
}

function getAllJointNames(model, readNameAt) {
  if (!model || !readNameAt) return [];
  const names = [];
  for (let i = 0; i < model.njnt; i++) {
    names.push(readNameAt(model.name_jntadr[i]));
  }
  return names;
}

function getAllBodyNames(model, readNameAt) {
  if (!model || !readNameAt) return [];
  const names = [];
  for (let i = 0; i < model.nbody; i++) {
    names.push(readNameAt(model.name_bodyadr[i]));
  }
  return names;
}

function matchRobotByXml(xmlText) {
  const includes = extractIncludeFiles(xmlText);
  if (!includes.length) return null;
  for (const config of Object.values(ROBOT_CONFIGS)) {
    const tokens = config.match?.includes || [];
    if (!tokens.length) continue;
    const matched = includes.some((file) => tokens.some((token) => file.includes(token)));
    if (matched) return config.key;
  }
  return null;
}

function matchRobotByModel(model, readNameAt) {
  const jointNames = getAllJointNames(model, readNameAt);
  if (!jointNames.length) return null;
  for (const config of Object.values(ROBOT_CONFIGS)) {
    const match = config.match || {};
    if (match.jointNames && match.jointNames.every((name) => jointNames.includes(name))) {
      return config.key;
    }
    if (match.jointPrefix && jointNames.some((name) => name.startsWith(match.jointPrefix))) {
      return config.key;
    }
  }
  return null;
}

function resolveEndEffectorName(config, model, readNameAt) {
  const bodyNames = getAllBodyNames(model, readNameAt);
  const candidates = [
    config.controlConfig?.endEffectorBodyName,
    ...(config.endEffectorCandidates || []),
  ].filter(Boolean);
  for (const candidate of candidates) {
    if (bodyNames.includes(candidate)) return candidate;
  }
  return null;
}

function resolveJointNames(config, model, readNameAt) {
  const allJointNames = getAllJointNames(model, readNameAt);
  const configured = config.controlConfig?.jointNames || [];
  
  // 1. Exact match
  const filtered = configured.filter((name) => allJointNames.includes(name));
  if (filtered.length === configured.length) return filtered;

  // 2. Prefix match
  const prefix = config.match?.jointPrefix;
  if (prefix) {
    const prefixed = allJointNames.filter((name) => name.startsWith(prefix));
    if (prefixed.length > 0) return prefixed;
  }

  // 3. Suffix match (Fuzzy fallback for Gen3 and others)
  // If configured names are present as suffixes in allJointNames, use them.
  // This handles cases where MuJoCo adds prefixes like "gen3/" automatically.
  const fuzzyMatched = [];
  const foundSuffixes = new Set();
  
  for (const configName of configured) {
    // Find a joint in allJointNames that ends with "/" + configName or is exactly configName
    const match = allJointNames.find(
      (name) => name === configName || name.endsWith('/' + configName)
    );
    if (match) {
      fuzzyMatched.push(match);
      foundSuffixes.add(configName);
    }
  }

  // Only return fuzzy matches if we found a significant number of them (e.g. all)
  // to avoid partial matching accidents.
  if (fuzzyMatched.length === configured.length) {
    return fuzzyMatched;
  }

  // 4. Return whatever we found in exact match if fuzzy failed
  return filtered.length > 0 ? filtered : configured;
}

function normalizeRobotConfig(config) {
  return {
    ...DEFAULT_ROBOT_CONFIG,
    ...config,
    controlConfig: {
      ...DEFAULT_CONTROL_CONFIG,
      ...(config?.controlConfig || {}),
    },
    gripper: {
      ...DEFAULT_ROBOT_CONFIG.gripper,
      ...(config?.gripper || {}),
    },
  };
}

function resolveRobotConfig({ xmlText, model, readNameAt }) {
  const byXml = matchRobotByXml(xmlText);
  const byModel = matchRobotByModel(model, readNameAt);
  const key = byXml || byModel || DEFAULT_ROBOT_CONFIG.key;
  const rawConfig = ROBOT_CONFIGS[key] || DEFAULT_ROBOT_CONFIG;
  const config = normalizeRobotConfig(rawConfig);

  const endEffectorBodyName = resolveEndEffectorName(config, model, readNameAt);
  const jointNames = resolveJointNames(config, model, readNameAt);
  const controlConfig = {
    ...config.controlConfig,
    endEffectorBodyName,
    jointNames,
  };

  return {
    ...config,
    controlConfig,
  };
}

export { resolveRobotConfig };
