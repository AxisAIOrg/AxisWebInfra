import * as THREE           from 'three';
import { GUI              } from 'three/addons/libs/lil-gui.module.min.js';
import { OrbitControls    } from 'three/addons/controls/OrbitControls.js';
import { KeyboardStateManager } from './utils/KeyboardStateManager.js';
import { IKController } from './utils/IKController.js';
import { setupGUI, loadSceneFromURL, drawTendonsAndFlex, getPosition, getQuaternion, toMujocoPos, standardNormal } from './mujocoUtils.js';
import { CheckerManager } from './checkers/CheckerManager.js';
import { loadMujoco }       from '@/lib/mujoco-utils';
import { unzipSync } from "fflate";

const initialScene = "scenes/close_box.xml";
const LOCAL_SCENE_BASE_URL = "/mujoco-assets/";
const FRANKA_SCENE_BASE_URL = "/mujoco-assets/robots/";
const FRANKA_SCENE_FILES = [
  "franka_emika_panda/LICENSE",
  "franka_emika_panda/assets/finger_0.obj",
  "franka_emika_panda/assets/finger_1.obj",
  "franka_emika_panda/assets/hand.stl",
  "franka_emika_panda/assets/hand_0.obj",
  "franka_emika_panda/assets/hand_1.obj",
  "franka_emika_panda/assets/hand_2.obj",
  "franka_emika_panda/assets/hand_3.obj",
  "franka_emika_panda/assets/hand_4.obj",
  "franka_emika_panda/assets/link0.stl",
  "franka_emika_panda/assets/link0_0.obj",
  "franka_emika_panda/assets/link0_1.obj",
  "franka_emika_panda/assets/link0_10.obj",
  "franka_emika_panda/assets/link0_11.obj",
  "franka_emika_panda/assets/link0_2.obj",
  "franka_emika_panda/assets/link0_3.obj",
  "franka_emika_panda/assets/link0_4.obj",
  "franka_emika_panda/assets/link0_5.obj",
  "franka_emika_panda/assets/link0_7.obj",
  "franka_emika_panda/assets/link0_8.obj",
  "franka_emika_panda/assets/link0_9.obj",
  "franka_emika_panda/assets/link1.obj",
  "franka_emika_panda/assets/link1.stl",
  "franka_emika_panda/assets/link2.obj",
  "franka_emika_panda/assets/link2.stl",
  "franka_emika_panda/assets/link3.stl",
  "franka_emika_panda/assets/link3_0.obj",
  "franka_emika_panda/assets/link3_1.obj",
  "franka_emika_panda/assets/link3_2.obj",
  "franka_emika_panda/assets/link3_3.obj",
  "franka_emika_panda/assets/link4.stl",
  "franka_emika_panda/assets/link4_0.obj",
  "franka_emika_panda/assets/link4_1.obj",
  "franka_emika_panda/assets/link4_2.obj",
  "franka_emika_panda/assets/link4_3.obj",
  "franka_emika_panda/assets/link5_0.obj",
  "franka_emika_panda/assets/link5_1.obj",
  "franka_emika_panda/assets/link5_2.obj",
  "franka_emika_panda/assets/link5_collision_0.obj",
  "franka_emika_panda/assets/link5_collision_1.obj",
  "franka_emika_panda/assets/link5_collision_2.obj",
  "franka_emika_panda/assets/link6.stl",
  "franka_emika_panda/assets/link6_0.obj",
  "franka_emika_panda/assets/link6_1.obj",
  "franka_emika_panda/assets/link6_10.obj",
  "franka_emika_panda/assets/link6_11.obj",
  "franka_emika_panda/assets/link6_12.obj",
  "franka_emika_panda/assets/link6_13.obj",
  "franka_emika_panda/assets/link6_14.obj",
  "franka_emika_panda/assets/link6_15.obj",
  "franka_emika_panda/assets/link6_16.obj",
  "franka_emika_panda/assets/link6_2.obj",
  "franka_emika_panda/assets/link6_3.obj",
  "franka_emika_panda/assets/link6_4.obj",
  "franka_emika_panda/assets/link6_5.obj",
  "franka_emika_panda/assets/link6_6.obj",
  "franka_emika_panda/assets/link6_7.obj",
  "franka_emika_panda/assets/link6_8.obj",
  "franka_emika_panda/assets/link6_9.obj",
  "franka_emika_panda/assets/link7.stl",
  "franka_emika_panda/assets/link7_0.obj",
  "franka_emika_panda/assets/link7_1.obj",
  "franka_emika_panda/assets/link7_2.obj",
  "franka_emika_panda/assets/link7_3.obj",
  "franka_emika_panda/assets/link7_4.obj",
  "franka_emika_panda/assets/link7_5.obj",
  "franka_emika_panda/assets/link7_6.obj",
  "franka_emika_panda/assets/link7_7.obj",
];

const FRANKA_READY_QPOS = new Float64Array([
  0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398, 0.04, 0.04
]);

const ASSET_ZIP_URL = process.env.NEXT_PUBLIC_MUJOCO_ASSET_ZIP_URL || "";
const ASSET_ZIP_ONLY = process.env.NEXT_PUBLIC_MUJOCO_ASSET_ZIP_ONLY === "true";
const assetZipIndexCache = new Map();

function normalizeZipPath(path) {
  const raw = String(path || "").replace(/\\/g, "/").replace(/^\.?\//, "");
  const parts = raw.split("/").filter((part) => part.length > 0);
  const stack = [];
  for (const part of parts) {
    if (part === ".") {
      continue;
    }
    if (part === "..") {
      stack.pop();
      continue;
    }
    stack.push(part);
  }
  return stack.join("/");
}

function buildZipCandidates(file, baseURL) {
  const normalizedFile = normalizeZipPath(file);
  const base = normalizeZipPath(String(baseURL || "").replace(/^\/+/, ""));
  const combined = base ? normalizeZipPath(`${base.replace(/\/+$/, "")}/${normalizedFile}`) : normalizedFile;
  const candidates = new Set([combined, normalizedFile]);
  for (const candidate of [combined, normalizedFile]) {
    if (candidate.startsWith("mujoco-assets/")) {
      candidates.add(candidate.slice("mujoco-assets/".length));
    }
  }
  return Array.from(candidates);
}

async function getAssetZipIndex() {
  if (!ASSET_ZIP_URL) {
    return null;
  }
  const cached = assetZipIndexCache.get(ASSET_ZIP_URL);
  if (cached) {
    return cached;
  }
  const response = await fetch(ASSET_ZIP_URL);
  if (!response.ok) {
    throw new Error(`Failed to fetch asset zip ${ASSET_ZIP_URL}: ${response.status} ${response.statusText}`);
  }
  const arrayBuffer = await response.arrayBuffer();
  const entries = unzipSync(new Uint8Array(arrayBuffer));
  const index = new Map();
  for (const [name, data] of Object.entries(entries)) {
    index.set(normalizeZipPath(name), data);
  }
  assetZipIndexCache.set(ASSET_ZIP_URL, index);
  return index;
}

function resolveZipEntry(zipIndex, file, baseURL) {
  if (!zipIndex) return null;
  const candidates = buildZipCandidates(file, baseURL);
  for (const candidate of candidates) {
    const entry = zipIndex.get(candidate);
    if (entry) {
      return entry;
    }
  }
  return null;
}

function ensureFSPath(mujoco, fsPath) {
  const parentIndex = fsPath.lastIndexOf('/');
  if (parentIndex <= 0) {
    return;
  }
  const dirPath = fsPath.substring(0, parentIndex);
  const segments = dirPath.split('/').filter(Boolean);
  let current = '';
  for (const segment of segments) {
    current += '/' + segment;
    let info;
    try {
      info = mujoco.FS.analyzePath(current);
    } catch (error) {
      info = { exists: false };
    }
    if (!info.exists) {
      mujoco.FS.mkdir(current);
    }
  }
}

async function preloadLocalSceneAssets(
  mujoco,
  files,
  baseURL = LOCAL_SCENE_BASE_URL,
  onProgress
) {
  const total = Array.isArray(files) ? files.length : 0;
  let loaded = 0;
  let zipIndex = null;
  if (ASSET_ZIP_URL) {
    try {
      zipIndex = await getAssetZipIndex();
    } catch (error) {
      console.warn(`[MuJoCoDemo] Failed to load asset zip ${ASSET_ZIP_URL}, falling back.`, error);
    }
  }
  if (ASSET_ZIP_ONLY && !zipIndex) {
    throw new Error(`zipOnly enabled but asset zip could not be loaded: ${ASSET_ZIP_URL || "(missing url)"}`);
  }
  for (const file of files) {
    if (typeof onProgress === "function") {
      onProgress({
        phase: "assets",
        message: `Loading assets (${loaded}/${total})`,
        loaded,
        total,
        currentFile: file,
        percent: total > 0 ? Math.round((loaded / total) * 100) : undefined,
      });
    }
    const targetPath = "/working/" + file;
    ensureFSPath(mujoco, targetPath);

    let payload = resolveZipEntry(zipIndex, file, baseURL);
    if (!payload) {
      if (ASSET_ZIP_ONLY) {
        throw new Error(`Asset not found in zip: ${file}`);
      }
      const url = baseURL + file;
      const response = await fetch(url);
      if (!response.ok) {
        throw new Error(`Failed to load ${url}: ${response.status} ${response.statusText}`);
      }
      const contentType = response.headers.get('content-type');
      if (contentType && contentType.includes('text/html')) {
        const text = await response.text();
        if (text.trim().startsWith('<!DOCTYPE') || text.trim().startsWith('<html')) {
          throw new Error(`Got HTML instead of file for ${url}. File may not exist on server.`);
        }
      }
      if (file.endsWith(".png") || file.endsWith(".stl") || file.endsWith(".msh")) {
        payload = new Uint8Array(await response.arrayBuffer());
      } else {
        payload = await response.text();
        if (file.endsWith(".xml") && !payload.trim().startsWith('<')) {
          throw new Error(`File ${url} does not appear to be valid XML. First 100 chars: ${payload.substring(0, 100)}`);
        }
      }
    }
    mujoco.FS.writeFile(targetPath, payload);
    loaded += 1;
    if (typeof onProgress === "function") {
      onProgress({
        phase: "assets",
        message: `Loading assets (${loaded}/${total})`,
        loaded,
        total,
        currentFile: file,
        percent: total > 0 ? Math.round((loaded / total) * 100) : undefined,
      });
    }
  }
}

function extractReferencedFilesFromXML(xmlText) {
  const files = new Set();
  const regex = /file="([^"]+)"/g;
  let match;
  while ((match = regex.exec(xmlText)) !== null) {
    if (match[1]) {
      files.add(match[1].trim());
    }
  }
  return Array.from(files);
}

function normalizeRelativePath(pathLike) {
  if (!pathLike) { return ""; }
  let normalized = pathLike.replace(/^\.\//, "");
  normalized = normalized.replace(/^\/+/, "");
  return normalized;
}

function parseDomainRandomizationFromXML(xmlText) {
  if (!xmlText || typeof xmlText !== "string") return null;
  try {
    const parser = new DOMParser();
    const doc = parser.parseFromString(xmlText, "text/xml");
    const textNodes = doc.getElementsByTagName("text");
    for (const node of Array.from(textNodes)) {
      const name = node.getAttribute("name") || "";
      if (name === "axis:domain_randomization" || name === "axis:dr") {
        const data = node.getAttribute("data") || "";
        if (!data) return null;
        return JSON.parse(data);
      }
    }
  } catch (error) {
    console.warn("[DomainRandomization] Failed to parse XML metadata:", error);
  }
  return null;
}

function getPathDir(pathLike) {
  const index = pathLike.lastIndexOf("/");
  if (index <= 0) return "";
  return pathLike.substring(0, index);
}

async function loadXMLFromPath(mujoco, filePath, onProgress) {
  const targetPath = "/working/" + filePath;
  try {
    mujoco.FS.stat(targetPath);
    const payload = mujoco.FS.readFile(targetPath, { encoding: "utf8" });
    if (typeof payload === "string" && payload.trim().startsWith("<")) {
      return payload;
    }
  } catch (e) {
    // Not in VFS, fall back to fetching/zip
  }

  let zipIndex = null;
  if (ASSET_ZIP_URL) {
    try {
      zipIndex = await getAssetZipIndex();
    } catch (error) {
      console.warn(`[MuJoCoDemo] Failed to load asset zip ${ASSET_ZIP_URL}, falling back.`, error);
    }
  }
  const zipPayload = resolveZipEntry(zipIndex, filePath, LOCAL_SCENE_BASE_URL);
  let xmlText = null;
  if (zipPayload) {
    xmlText = new TextDecoder("utf-8").decode(zipPayload);
  } else {
    if (ASSET_ZIP_ONLY) {
      throw new Error(`Scene XML not found in zip: ${filePath}`);
    }
    const sceneURL = LOCAL_SCENE_BASE_URL + filePath;
    const sceneResponse = await fetch(sceneURL);
    if (!sceneResponse.ok) {
      throw new Error(`Failed to load scene XML ${sceneURL}: ${sceneResponse.status} ${sceneResponse.statusText}`);
    }
    xmlText = await sceneResponse.text();
  }
  if (!xmlText.trim().startsWith("<")) {
    throw new Error(`Scene XML ${filePath} does not appear to be valid XML. First 100 chars: ${xmlText.substring(0, 100)}`);
  }
  ensureFSPath(mujoco, targetPath);
  mujoco.FS.writeFile(targetPath, xmlText);
  if (typeof onProgress === "function") {
    onProgress({
      phase: "scene_xml",
      message: `Loaded XML: ${filePath}`,
    });
  }
  return xmlText;
}

async function collectAssetPathsRecursive(mujoco, entryPath, onProgress) {
  const pending = [entryPath];
  const seenXml = new Set();
  const assetPaths = new Set();

  while (pending.length > 0) {
    const xmlPath = pending.pop();
    if (!xmlPath || seenXml.has(xmlPath)) continue;
    seenXml.add(xmlPath);

    const xmlText = await loadXMLFromPath(mujoco, xmlPath, onProgress);
    const referencedFiles = extractReferencedFilesFromXML(xmlText);
    const xmlDir = getPathDir(xmlPath);

    for (const relativePath of referencedFiles) {
      const normalizedRelative = normalizeRelativePath(relativePath);
      if (!normalizedRelative || normalizedRelative.toLowerCase().startsWith("http")) {
        continue;
      }
      const assetPath = xmlDir ? `${xmlDir}/${normalizedRelative}` : normalizedRelative;
      if (assetPath.endsWith(".xml") || assetPath.endsWith(".mjcf")) {
        pending.push(assetPath);
      } else {
        assetPaths.add(assetPath);
      }
    }
  }

  return Array.from(assetPaths);
}

async function preloadSceneWithAssets(mujoco, scenePath, onProgress) {
  if (typeof onProgress === "function") {
    onProgress({
      phase: "scene_xml",
      message: `Loading scene XML: ${scenePath}`,
      percent: 0,
    });
  }
  const assetPaths = await collectAssetPathsRecursive(mujoco, scenePath, onProgress);

  if (typeof onProgress === "function") {
    onProgress({
      phase: "assets",
      message: `Loading assets (0/${assetPaths.length})`,
      loaded: 0,
      total: assetPaths.length,
      percent: assetPaths.length > 0 ? 0 : 100,
    });
  }

  if (assetPaths.length > 0) {
    await preloadLocalSceneAssets(mujoco, assetPaths, LOCAL_SCENE_BASE_URL, onProgress);
  }
}

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
    this.gripperFingerJointNames = ['franka/panda_finger_joint1', 'franka/panda_finger_joint2'];
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
    this.robotControlConfig = {
      endEffectorBodyName: 'franka/panda_hand',
      jointNames: ['franka/panda_joint1', 'franka/panda_joint2', 'franka/panda_joint3', 
                   'franka/panda_joint4', 'franka/panda_joint5', 'franka/panda_joint6', 
                   'franka/panda_joint7'],
      mode: 'joint',
      translationGain: 1.5,
      rotationGain: 0.8,
      maxIterations: 6,
      stepLimit: 0.04,
    };
    this.initialState = null;
    this.guiPauseController = null;
    this.isSavingEpisode = false;
    
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

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    // Get container dimensions instead of window dimensions
    const containerWidth = this.container.clientWidth || parentElement.clientWidth || window.innerWidth;
    const containerHeight = this.container.clientHeight || parentElement.clientHeight || window.innerHeight;

    this.camera = new THREE.PerspectiveCamera( 45, containerWidth / containerHeight, 0.001, 100 );
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(2.0, 1.7, 1.7);
    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5 );

    this.ambientLight = new THREE.AmbientLight( 0xffffff, 0.1 * 3.14 );
    this.ambientLight.name = 'AmbientLight';
    this.scene.add( this.ambientLight );

    this.spotlight = new THREE.SpotLight();
    this.spotlight.angle = 1.11;
    this.spotlight.distance = 10000;
    this.spotlight.penumbra = 0.5;
    this.spotlight.castShadow = true;
    this.spotlight.intensity = this.spotlight.intensity * 3.14 * 10.0;
    this.spotlight.shadow.mapSize.width = 1024;
    this.spotlight.shadow.mapSize.height = 1024;
    this.spotlight.shadow.camera.near = 0.1;
    this.spotlight.shadow.camera.far = 100;
    this.spotlight.position.set(0, 3, 3);
    const targetObject = new THREE.Object3D();
    this.scene.add(targetObject);
    this.spotlight.target = targetObject;
    targetObject.position.set(0, 1, 0);
    this.scene.add( this.spotlight );

    this.renderer = new THREE.WebGLRenderer( { antialias: true } );
    this.renderer.setPixelRatio(1.0);
    // Use container dimensions instead of window dimensions
    this.renderer.setSize( containerWidth, containerHeight );
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    THREE.ColorManagement.enabled = false;
    this.renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
    this.renderer.useLegacyLights = true;
    
    // Ensure canvas doesn't break out of container
    this.renderer.domElement.style.display = 'block';
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.renderer.domElement.style.maxWidth = '100%';
    this.renderer.domElement.style.maxHeight = '100%';

    this.renderer.setAnimationLoop( this.render.bind(this) );

    this.container.appendChild( this.renderer.domElement );

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.7, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    // Use ResizeObserver to watch container size changes (more accurate than window resize)
    if (typeof ResizeObserver !== 'undefined') {
      this.resizeObserver = new ResizeObserver(() => {
        this.onWindowResize();
      });
      this.resizeObserver.observe(this.container);
    } else {
      // Fallback to window resize for older browsers
      window.addEventListener('resize', this.onWindowResize.bind(this));
    }
    // Drag interaction intentionally disabled.
    // this.dragStateManager = new DragStateManager(...);

    this.keyboardStateManager = new KeyboardStateManager({
      // Teleop speed tuning (user request):
      // - translation speed x4
      // - rotation speed x5
      translationSpeed: 0.25 * 4,
      rotationSpeed: THREE.MathUtils.degToRad(45) * 5,
      onTranslate: this.handleTeleopTranslate.bind(this),
      onRotate: this.handleTeleopRotate.bind(this),
      // Space toggles gripper open/close (no need to hold)
      gripperMode: 'toggle',
      onGripperCommand: this.toggleGripper.bind(this),
      onCompleteEpisode: this.completeEpisode.bind(this),
      onResetEpisode: this.resetEpisode.bind(this),
      onSaveAndExit: this.saveAndExit.bind(this),
      onSaveCheckpoint: this.saveCheckpoint.bind(this),
      onRestoreCheckpoint: this.restoreCheckpoint.bind(this),
      onReplayTrajectory: this.toggleReplayTrajectory.bind(this),
    });
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
    try {
      // Lazily load MuJoCo WASM and set up the virtual filesystem
      if (!this.mujoco) {
        this.emitLoadingProgress({
          phase: "mujoco",
          message: "Loading MuJoCo runtime…",
          percent: 5,
        });
        const mujoco = await loadMujoco();
        this.mujoco = mujoco;
        try {
          mujoco.FS.mkdir('/working');
        } catch (e) {
          // directory may already exist
        }
        mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
        this.emitLoadingProgress({
          phase: "assets",
          message: "Preloading base scene assets…",
          percent: 10,
        });
        await preloadSceneWithAssets(mujoco, initialScene, this.emitLoadingProgress.bind(this));
      }

      const mujoco = this.mujoco;

      // Determine which scene to load
      let sceneFileToLoad = initialScene;
      
      // 1. Check URL Params for task id (use "id" to align with page.tsx)
      const urlParams = new URLSearchParams(window.location.search);
      const taskId = urlParams.get('id');
      
      if (taskId) {
        console.log(`[main] Found taskId=${taskId} in URL, fetching from API...`);
        try {
          // Use API base URL from window (injected by HTML) or fallback to /api
          const apiBase = window.MUJOCO_API_BASE || '/api';
          const apiUrl = `${apiBase}/tasks/${taskId}`;
          console.log(`[main] Fetching task from: ${apiUrl}`);
          this.emitLoadingProgress({
            phase: "task_xml",
            message: "Fetching task XML…",
            percent: 15,
          });
          const response = await fetch(apiUrl);
          if (!response.ok) throw new Error(`API Error: ${response.status}`);
          const taskData = await response.json();
          
          if (taskData.mjcf_xml) {
            // Write to VFS in the same directory as the default scene so relative asset paths work
            // The default assets are loaded into /working/scenes/
            const dynamicFilename = `scenes/task_${taskId}.xml`;
            mujoco.FS.writeFile("/working/" + dynamicFilename, taskData.mjcf_xml);
            console.log(`[main] Wrote task XML to /working/${dynamicFilename}`);
            sceneFileToLoad = dynamicFilename;
            if (!this.domainRandomizationConfig) {
              const drConfig = parseDomainRandomizationFromXML(taskData.mjcf_xml);
              if (drConfig) {
                this.domainRandomizationConfig = drConfig;
              } else {
                this.domainRandomizationConfig = null;
              }
            }
          } else {
            console.warn("[main] Task has no XML, falling back...");
          }
        } catch (error) {
          console.error("[main] Failed to fetch task XML:", error);
          console.warn("[main] Fallback reason: Failed to fetch task XML from API.");
        }
      }

      // Load the selected scene
      this.params.scene = sceneFileToLoad;
      console.log(`[main] Loading scene file: ${sceneFileToLoad}`);
      // Preload the scene (and assets) we are about to load so VFS contains referenced files
      try {
        this.emitLoadingProgress({
          phase: "assets",
          message: "Preloading task scene assets…",
          percent: 25,
        });
        await preloadSceneWithAssets(mujoco, sceneFileToLoad, this.emitLoadingProgress.bind(this));
      } catch (error) {
        console.warn(`[main] Failed to preload ${sceneFileToLoad}: ${error.message}`);
      }
      try {
        this.emitLoadingProgress({
          phase: "model",
          message: "Parsing MuJoCo model…",
          percent: 90,
        });
        [this.model, this.data, this.bodies, this.lights] = await loadSceneFromURL(mujoco, sceneFileToLoad, this);
      } catch (error) {
        console.error(`[main] Failed to load ${sceneFileToLoad}, trying default fallback...`);
        console.warn(`[main] Fallback reason: ${error.message}`);
        this.emitLoadingProgress({
          phase: "model",
          message: "Parsing fallback MuJoCo model…",
          percent: 90,
        });
        [this.model, this.data, this.bodies, this.lights] = await loadSceneFromURL(mujoco, initialScene, this);
      }
      // Ensure name buffer is ready for actuator/joint lookups (used by replay conversion)
      this.refreshNameBuffer();

      this.setFrankaReadyPose();
      this.gui = new GUI();
      setupGUI(this);
      try {
        this.setupIKController();
      } catch (error) {
        console.error('[init] Failed to setup IKController:', error);
        this.ikController = null;
      }
      if (this.initialStateConfig) {
        console.log('[InitialState] Applying task initial_state...');
        this.applyInitialState(this.initialStateConfig);
      }
      this.captureInitialState();
      if (this.domainRandomizationConfig) {
        this.applyDomainRandomization(this.domainRandomizationConfig);
        this.mujoco.mj_forward(this.model, this.data);
        if (this.ikController) {
          this.ikController.syncCtrlFromQpos();
          this.ikController.syncTargetsFromModel();
        }
      }
      this.clearEpisodeState();
      this.updateSceneFromData();
      
      // Start status broadcasting for React integration
      this.startStatusBroadcasting();
      this.emitLoadingProgress({
        phase: "ready",
        message: "Ready",
        percent: 100,
      });
    } catch (error) {
      console.error('[init] Initialization error:', error);
      this.emitLoadingProgress({
        phase: "error",
        message: error?.message ? `Init failed: ${error.message}` : "Init failed",
      });
      throw error;
    }
  }

  setupReplay() {
    // Check for pending replay samples (from React parent or window.parent for iframe compatibility)
    let samples = null;
    if (this.pendingReplaySamples && this.pendingReplayFlag) {
      samples = this.pendingReplaySamples;
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
      this.pendingReplaySamples = samples;
      this.pendingReplayFlag = true;
      // Load and start replay
      this.loadTrajectoryFromData(samples);
    } else {
      console.log(`[setupReplay] No pending replay samples found`);
    }
  }

  onModelReloaded() {
    // IMPORTANT: refresh name buffer before any logic that reads body/joint/actuator names.
    // setupIKController/setupGripperControls/setupCheckers all depend on readNameAt().
    this.refreshNameBuffer();
    try {
      this.setupIKController();
    } catch (error) {
      console.error('[onModelReloaded] Failed to setup IKController:', error);
      this.ikController = null;
    }
    this.setupGripperControls();
    this.setupCheckers();
    this.gripperClosed = false;
    this.checkpointState = null;
    this.checkpointSavedAt = null;
    this.setFrankaReadyPose();
    if (this.initialStateConfig) {
      console.log('[InitialState] Applying task initial_state (onModelReloaded)...');
      this.applyInitialState(this.initialStateConfig);
    }
    this.captureInitialState();
    this.clearEpisodeState();
    this.params.paused = false;
    this.guiPauseController?.setValue(false);
    this.resetSuccessTimer();
    this.resetSimulation();
    
    // Setup replay if pending samples exist
    this.setupReplay();
  }

  setupCheckers() {
    // Reset
    this.checkerManager = null;

    if (!this.checkerConfig) {
      console.log('[Teleop] No checker config provided from task, skipping checker setup');
      return;
    }

    console.log('[Teleop] Setting up checkers from task config:', JSON.stringify(this.checkerConfig, null, 2));
    try {
      this.checkerManager = new CheckerManager(
        this.mujoco,
        this.model,
        this.data,
        this.checkerConfig,
        { keyboardStateManager: this.keyboardStateManager }
      );
      console.log('[Teleop] CheckerManager initialized from task config');
    } catch (error) {
      console.error('[Teleop] Failed to setup CheckerManager from task config:', error);
      this.checkerManager = null;
    }
  }

  setupIKController() {
    const allBodyNames = [];
    for (let i = 0; i < this.model.nbody; i++) {
      allBodyNames.push(this.readNameAt(this.model.name_bodyadr[i]));
    }
    const allJointNames = [];
    for (let i = 0; i < this.model.njnt; i++) {
      allJointNames.push(this.readNameAt(this.model.name_jntadr[i]));
    }
    const configuredEE = this.robotControlConfig.endEffectorBodyName;
    if (!allBodyNames.includes(configuredEE)) {
      throw new Error(`Required end effector body "${configuredEE}" not found in model.`);
    }
    const configuredJoints = this.robotControlConfig.jointNames;
    const existingJoints = configuredJoints.filter(j => allJointNames.includes(j));
    if (existingJoints.length === 0) {
      const frankaJoints = allJointNames.filter(n => n.startsWith('franka/panda_joint'));
      const jointsWithActuators = [];
      for (const jointName of frankaJoints) {
        let hasActuator = false;
        for (let a = 0; a < this.model.nu; a++) {
          const actuatorName = this.readNameAt(this.model.name_actuatoradr[a]);
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
        this.robotControlConfig.jointNames = jointsWithActuators.sort();
      } else {
        throw new Error('No franka joints with actuators found!');
      }
    } else if (existingJoints.length < configuredJoints.length) {
      this.robotControlConfig.jointNames = existingJoints;
    }
    try {
      this.ikController = new IKController(this.mujoco, this.model, this.data, this.robotControlConfig);
    } catch (error) {
      console.error('[Teleop] Failed to create IKController:', error);
      this.ikController = null;
    }
  }

  refreshNameBuffer() {
    if (!this.model) { return; }
    this.namesBuffer = new Uint8Array(this.model.names);
  }

  readNameAt(address) {
    if (!this.namesBuffer || address < 0 || address >= this.namesBuffer.length) { return ''; }
    let end = address;
    while (end < this.namesBuffer.length && this.namesBuffer[end] !== 0) { end++; }
    return this.textDecoder.decode(this.namesBuffer.subarray(address, end));
  }

  findIdByName(name, addressArray, count) {
    if (!name || !addressArray || count <= 0) { return -1; }
    for (let i = 0; i < count; i++) {
      const start = addressArray[i];
      const candidate = this.readNameAt(start);
      if (candidate === name) { return i; }
    }
    return -1;
  }

  getJointAddress(name) {
    const jointId = this.findIdByName(name, this.model.name_jntadr, this.model.njnt);
    if (jointId === -1) { return -1; }
    return this.model.jnt_qposadr[jointId];
  }

  setupGripperControls() {
    if (!this.model) { return; }
    this.gripperFingerJointAddresses = [];
    this.gripperFingerActuatorIndices = [];
    for (const jointName of this.gripperFingerJointNames) {
      const address = this.getJointAddress(jointName);
      if (address >= 0) {
        this.gripperFingerJointAddresses.push(address);
      }
      const actuatorIdx = this.findIdByName(jointName, this.model.name_actuatoradr, this.model.nu);
      if (actuatorIdx >= 0 && actuatorIdx < this.model.nu) {
        this.gripperFingerActuatorIndices.push(actuatorIdx);
      } else {
        for (let a = 0; a < this.model.nu; a++) {
          const actuatorName = this.readNameAt(this.model.name_actuatoradr[a]);
          if (actuatorName === jointName || 
              actuatorName.endsWith('/' + jointName) ||
              actuatorName.endsWith(jointName)) {
            this.gripperFingerActuatorIndices.push(a);
            break;
          }
        }
      }
    }
  }

  // ========== Task Initial State (data-driven) ==========
  applyInitialState(initialState) {
    if (!initialState || !this.mujoco || !this.model || !this.data) return;

    const mujoco = this.mujoco;

    const parseVec3 = (v) => {
      if (Array.isArray(v) && v.length >= 3) return { x: Number(v[0]), y: Number(v[1]), z: Number(v[2]) };
      return null;
    };
    const parseQuatWXYZ = (q) => {
      if (Array.isArray(q) && q.length >= 4) return { w: Number(q[0]), x: Number(q[1]), y: Number(q[2]), z: Number(q[3]) };
      return null;
    };

    const trySetJointScalarQpos = (jointName, value) => {
      const jointId = this.findIdByName(jointName, this.model.name_jntadr, this.model.njnt);
      if (jointId < 0) return false;
      const jointType = this.model.jnt_type[jointId];
      if (jointType === mujoco.mjtJoint.mjJNT_FREE.value) return false;
      const addr = this.model.jnt_qposadr[jointId];
      if (addr < 0 || addr >= this.data.qpos.length) return false;
      this.data.qpos[addr] = value;

      // Best-effort: also sync actuator ctrl if an actuator exists
      const directActuatorIdx = this.findIdByName(jointName, this.model.name_actuatoradr, this.model.nu);
      if (directActuatorIdx >= 0 && directActuatorIdx < this.data.ctrl.length) {
        this.data.ctrl[directActuatorIdx] = value;
        return true;
      }
      for (let a = 0; a < this.model.nu; a++) {
        const actuatorName = this.readNameAt(this.model.name_actuatoradr[a]);
        if (actuatorName === jointName || actuatorName.endsWith('/' + jointName) || actuatorName.endsWith(jointName)) {
          if (a < this.data.ctrl.length) this.data.ctrl[a] = value;
          break;
        }
      }
      return true;
    };

    const resolveJointNameCandidates = (entityKey, entityCfg, jointKey) => {
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
    };

    const trySetBodyFreeJointPose = (entityKey, entityCfg) => {
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
        const id = this.findIdByName(bn, this.model.name_bodyadr, this.model.nbody);
        if (id >= 0) { bodyId = id; break; }
      }
      if (bodyId < 0) return false;

      const mjJNT_FREE = mujoco.mjtJoint.mjJNT_FREE.value;
      let freeJointId = -1;
      for (let j = 0; j < this.model.njnt; j++) {
        if (this.model.jnt_bodyid[j] === bodyId && this.model.jnt_type[j] === mjJNT_FREE) {
          freeJointId = j;
          break;
        }
      }
      if (freeJointId < 0) {
        console.warn(`[InitialState] Body "${this.readNameAt(this.model.name_bodyadr[bodyId])}" has no FREE joint; cannot apply pos/rot at runtime.`);
        return false;
      }

      const addr = this.model.jnt_qposadr[freeJointId];
      if (addr < 0 || (addr + 6) >= this.data.qpos.length) return false;
      // MuJoCo free joint qpos layout: [x y z qw qx qy qz]
      this.data.qpos[addr + 0] = pos.x;
      this.data.qpos[addr + 1] = pos.y;
      this.data.qpos[addr + 2] = pos.z;
      this.data.qpos[addr + 3] = quat.w;
      this.data.qpos[addr + 4] = quat.x;
      this.data.qpos[addr + 5] = quat.y;
      this.data.qpos[addr + 6] = quat.z;
      return true;
    };

    const applyEntity = (entityKey, entityCfg) => {
      if (!entityCfg) return;

      // Pose (only if movable via FREE joint)
      trySetBodyFreeJointPose(entityKey, entityCfg);

      // Joint dof_pos
      const dofPos = entityCfg.dof_pos || entityCfg.dofPos || null;
      if (dofPos && typeof dofPos === 'object') {
        for (const [jointKey, value] of Object.entries(dofPos)) {
          const v = Number(value);
          if (!Number.isFinite(v)) continue;
          const candidates = resolveJointNameCandidates(entityKey, entityCfg, jointKey);
          let applied = false;
          for (const name of candidates) {
            if (trySetJointScalarQpos(name, v)) { applied = true; break; }
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

      mujoco.mj_forward(this.model, this.data);
      this.updateSceneFromData();

      if (this.ikController) {
        this.ikController.syncCtrlFromQpos();
        this.ikController.syncTargetsFromModel();
      }
    } catch (e) {
      console.warn('[InitialState] Failed to apply initial_state:', e);
    }
  }

  applyDomainRandomization(domainRandomization) {
    const cfg = domainRandomization || this.domainRandomizationConfig;
    if (!cfg || !this.mujoco || !this.model || !this.data) { return; }
    const mujoco = this.mujoco;

    const parseRange = (value) => {
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
    };

    const parseVec3Ranges = (value) => {
      if (!Array.isArray(value) || value.length !== 3) return null;
      const ranges = value.map(parseRange);
      if (ranges.some((r) => !r)) return null;
      return ranges;
    };

    const randomInRange = (range) => {
      const [min, max] = range;
      return min + Math.random() * (max - min);
    };

    const resolveJointNameCandidates = (entityKey, entityCfg, jointKey) => {
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
    };

    const trySetJointScalarQpos = (jointName, value, isDelta) => {
      if (!jointName) return false;
      const jointId = this.findIdByName(jointName, this.model.name_jntadr, this.model.njnt);
      if (jointId < 0) return false;
      const addr = this.model.jnt_qposadr[jointId];
      if (addr < 0 || addr >= this.data.qpos.length) return false;
      const target = isDelta ? this.data.qpos[addr] + value : value;
      this.data.qpos[addr] = target;

      const directActuatorIdx = this.findIdByName(jointName, this.model.name_actuatoradr, this.model.nu);
      if (directActuatorIdx >= 0 && directActuatorIdx < this.data.ctrl.length) {
        this.data.ctrl[directActuatorIdx] = target;
        return true;
      }
      for (let a = 0; a < this.model.nu; a++) {
        const actuatorName = this.readNameAt(this.model.name_actuatoradr[a]);
        if (actuatorName === jointName || actuatorName.endsWith('/' + jointName) || actuatorName.endsWith(jointName)) {
          if (a < this.data.ctrl.length) this.data.ctrl[a] = target;
          break;
        }
      }
      return true;
    };

    const tryRandomizeBodyFreeJoint = (entityKey, entityCfg, posRanges, posIsDelta, rotRanges, rotIsDelta) => {
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
        const id = this.findIdByName(bn, this.model.name_bodyadr, this.model.nbody);
        if (id >= 0) { bodyId = id; break; }
      }
      if (bodyId < 0) return false;

      const mjJNT_FREE = mujoco.mjtJoint.mjJNT_FREE.value;
      let freeJointId = -1;
      for (let j = 0; j < this.model.njnt; j++) {
        if (this.model.jnt_bodyid[j] === bodyId && this.model.jnt_type[j] === mjJNT_FREE) {
          freeJointId = j;
          break;
        }
      }
      if (freeJointId < 0) {
        console.warn(`[DomainRandomization] Body "${this.readNameAt(this.model.name_bodyadr[bodyId])}" has no FREE joint; cannot randomize pos.`);
        return false;
      }

      const addr = this.model.jnt_qposadr[freeJointId];
      if (addr < 0 || (addr + 6) >= this.data.qpos.length) return false;
      if (posRanges) {
        const current = {
          x: this.data.qpos[addr + 0],
          y: this.data.qpos[addr + 1],
          z: this.data.qpos[addr + 2],
        };
        const newPos = {
          x: posIsDelta ? current.x + randomInRange(posRanges[0]) : randomInRange(posRanges[0]),
          y: posIsDelta ? current.y + randomInRange(posRanges[1]) : randomInRange(posRanges[1]),
          z: posIsDelta ? current.z + randomInRange(posRanges[2]) : randomInRange(posRanges[2]),
        };
        this.data.qpos[addr + 0] = newPos.x;
        this.data.qpos[addr + 1] = newPos.y;
        this.data.qpos[addr + 2] = newPos.z;
      }
      if (rotRanges) {
        const currentQuat = new THREE.Quaternion(
          this.data.qpos[addr + 4],
          this.data.qpos[addr + 5],
          this.data.qpos[addr + 6],
          this.data.qpos[addr + 3]
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
        this.data.qpos[addr + 3] = nextQuat.w;
        this.data.qpos[addr + 4] = nextQuat.x;
        this.data.qpos[addr + 5] = nextQuat.y;
        this.data.qpos[addr + 6] = nextQuat.z;
      }
      return true;
    };

    const applyJointMap = (entityKey, entityCfg, jointMap, isDelta) => {
      if (!jointMap || typeof jointMap !== "object") return;
      for (const [jointKey, rawRange] of Object.entries(jointMap)) {
        const range = parseRange(rawRange);
        if (!range) continue;
        const value = randomInRange(range);
        const candidates = resolveJointNameCandidates(entityKey, entityCfg, jointKey);
        let applied = false;
        for (const name of candidates) {
          if (trySetJointScalarQpos(name, value, isDelta)) { applied = true; break; }
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
        tryRandomizeBodyFreeJoint(entityKey, entityCfg, posRanges, !!posDelta, rotRanges, !!rotDelta);
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

  setFrankaReadyPose() {
    if (!this.model || !this.data) { return; }
    const frankaJointNames = [
      'franka/panda_joint1', 'franka/panda_joint2', 'franka/panda_joint3',
      'franka/panda_joint4', 'franka/panda_joint5', 'franka/panda_joint6',
      'franka/panda_joint7', 'franka/panda_finger_joint1', 'franka/panda_finger_joint2'
    ];
    for (let i = 0; i < frankaJointNames.length && i < FRANKA_READY_QPOS.length; i++) {
      const jointName = frankaJointNames[i];
      const address = this.getJointAddress(jointName);
      if (address >= 0 && address < this.data.qpos.length) {
        const targetQpos = FRANKA_READY_QPOS[i];
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
    return new THREE.Vector3(delta.x, delta.z, -delta.y);
  }

  handleTeleopTranslate(delta) {
    if (this.ikController) {
      const alignedDelta = this.alignMovementToBase(delta);
      this.ikController.adjustTargetPosition(alignedDelta);
    }
  }

  handleTeleopRotate(deltaEuler) {
    if (this.ikController) {
      this.ikController.adjustTargetOrientation(deltaEuler);
    }
  }

  setGripperState(closed) {
    if (!this.model || !this.data) { return; }
    if (this.gripperClosed === closed) { return; }
    this.gripperClosed = closed;
    const poseValue = closed ? this.gripperClosedPose : this.gripperOpenPose;
    // IMPORTANT:
    // - Prefer actuator control (data.ctrl). This allows contact constraints to prevent penetration.
    // - Writing qpos directly will "teleport" fingers through objects and causes gripper penetration.
    const hasActuators = this.gripperFingerActuatorIndices.length > 0;
    if (hasActuators) {
    for (let i = 0; i < this.gripperFingerActuatorIndices.length; i++) {
      const actuatorIdx = this.gripperFingerActuatorIndices[i];
      if (actuatorIdx >= 0 && actuatorIdx < this.data.ctrl.length) {
        this.data.ctrl[actuatorIdx] = poseValue;
      }
    }
    } else {
      // Fallback only if actuators are missing (should be rare for Franka models)
    for (const address of this.gripperFingerJointAddresses) {
      if (address >= 0 && address < this.data.qpos.length) {
        this.data.qpos[address] = poseValue;
        }
      }
    }
    this.mujoco.mj_forward(this.model, this.data);
  }

  async toggleReplayTrajectory() {
    if (this.replayPlaying) {
      this.stopReplayTrajectory();
      return;
    }
    await this.startReplayTrajectory();
  }

  async startReplayTrajectory() {
    if (!this.lastSavedTrajectoryPath) { return; }
    const url = this.getTrajectoryFetchURL(this.lastSavedTrajectoryPath);
    if (!url) { return; }
    try {
      const response = await fetch(url, { cache: 'no-store' });
      if (!response.ok) { return; }
      const payload = await response.json();
      if (!Array.isArray(payload.samples) || !payload.samples.length) { return; }
      this.loadTrajectoryFromData(payload.samples);
    } catch (error) {
      console.error('[Teleop] Failed to load trajectory:', error);
    }
  }

  loadTrajectoryFromData(samples) {
    if (!Array.isArray(samples) || !samples.length) {
      console.error('[Teleop] Invalid trajectory samples:', samples);
      return;
    }
    if (!this.model || !this.data) {
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
      const qpos = Array.from(this.data.qpos);
      
      if (sample.robot_joints && typeof sample.robot_joints === 'object') {
        let mappedCount = 0;
        
        // Map each joint name to its qpos address and set the value
        for (const [jointName, value] of Object.entries(sample.robot_joints)) {
          const address = this.getJointAddress(jointName);
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
        const qvel = this.data.qvel ? Array.from(this.data.qvel) : [];
        
        // Map each joint name to its qvel address
        for (const [jointName, value] of Object.entries(sample.robot_velocities)) {
          const jointId = this.findIdByName(jointName, this.model.name_jntadr, this.model.njnt);
          if (jointId >= 0) {
            const qvelAddress = this.model.jnt_dofadr[jointId];
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
        if (!this.namesBuffer) {
          this.refreshNameBuffer();
        }
        const ctrl = this.data.ctrl ? Array.from(this.data.ctrl) : [];
        
        // Map actuator names to ctrl indices
        for (let i = 0; i < this.model.nu; i++) {
          const actuatorName = this.readNameAt(this.model.name_actuatoradr[i]);
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
    this.clearEpisodeState();
    
    // 2. Reset success flags
    this.successAchieved = false;
    this.successTime = null;
    this.resetSuccessTimer();
    
    // 3. Reset simulation (this does restoreInitialState, mj_forward, updateSceneFromData, IK sync)
    this.resetSimulation();
    
    // 4. Unpause
    this.params.paused = false;
    this.guiPauseController?.setValue(false);
    
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

    this.replaySamples = validSamples;
    this.replayIndex = 0;
    this.replayStartTime = null;
    this.replayPlaying = true;
    this.replayLoopLogged = false; // log once when render enters replay loop
    
    // Reset sequence above already unpaused the simulation
    // The replay loop will naturally apply samples starting from index 0
    
    console.log(`[Teleop] Successfully loaded ${validSamples.length} trajectory samples for replay`);
    console.log(`[Teleop] Replay state: playing=${this.replayPlaying}, samples=${this.replaySamples.length}, first elapsedMs=${this.replaySamples[0]?.elapsedMs}`);
    console.log(`[Teleop] Simulation paused=${this.params.paused}, model=${!!this.model}, data=${!!this.data}`);
    } catch (error) {
      console.error('[Teleop] Error during trajectory conversion:', error);
      console.error('[Teleop] Error stack:', error.stack);
      this.replayPlaying = false;
      this.replaySamples = null;
    }
  }

  stopReplayTrajectory() {
    this.replayPlaying = false;
    this.replaySamples = null;
    this.replayIndex = 0;
    this.replayStartTime = null;
  }

  getTrajectoryFetchURL(path) {
    if (!path) { return null; }
    if (path.startsWith('http://') || path.startsWith('https://') || path.startsWith('/')) {
      return path;
    }
    return '/' + path;
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
    if (!this.replayPlaying || !this.replaySamples?.length) { 
      if (!this.replayPlaying) {
        console.warn('[Teleop] updateReplay called but replayPlaying is false');
      }
      if (!this.replaySamples?.length) {
        console.warn('[Teleop] updateReplay called but no replay samples');
      }
      return; 
    }
    if (this.replayStartTime === null) {
      this.replayStartTime = timeMS;
      console.log(`[Teleop] Replay started at timeMS=${timeMS}`);
    }
    const elapsed = timeMS - this.replayStartTime;
    let appliedCount = 0;
    while (this.replayIndex < this.replaySamples.length &&
           this.replaySamples[this.replayIndex].elapsedMs <= elapsed) {
      this.applyTrajectorySample(this.replaySamples[this.replayIndex]);
      this.replayIndex++;
      appliedCount++;
    }
    if (appliedCount > 0 && this.replayIndex % 10 === 0) {
      console.log(`[Teleop] Replay progress: ${this.replayIndex}/${this.replaySamples.length} samples (elapsed: ${elapsed.toFixed(1)}ms)`);
    }
    if (this.replayIndex >= this.replaySamples.length) {
      console.log('[Teleop] Replay completed!');
      this.stopReplayTrajectory();
    }
  }

  applyTrajectorySample(sample) {
    if (!sample || !this.data) { 
      console.warn('[Teleop] applyTrajectorySample: missing sample or data', { hasSample: !!sample, hasData: !!this.data });
      return; 
    }
    let applied = false;
    if (sample.qpos && this.data.qpos.length >= sample.qpos.length) {
      this.data.qpos.set(sample.qpos);
      applied = true;
    }
    if (sample.qvel && this.data.qvel?.length >= sample.qvel.length) {
      this.data.qvel.set(sample.qvel);
    }
    if (sample.ctrl && this.data.ctrl?.length >= sample.ctrl.length) {
      this.data.ctrl.set(sample.ctrl);
    }
    if (sample.mocap_pos && this.model.nmocap > 0) {
      this.data.mocap_pos.set(sample.mocap_pos);
    }
    if (sample.mocap_quat && this.model.nmocap > 0) {
      this.data.mocap_quat.set(sample.mocap_quat);
    }
    if (applied) {
      this.mujoco.mj_forward(this.model, this.data);
      this.updateSceneFromData();
    } else {
      console.warn('[Teleop] applyTrajectorySample: no qpos applied', { 
        hasQpos: !!sample.qpos, 
        qposLength: sample.qpos?.length, 
        dataQposLength: this.data.qpos.length 
      });
    }
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
    this.setFrankaReadyPose();
    this.captureInitialState();
    this.resetSimulation();
    this.setFrankaReadyPose();
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
    // Export trajectory exactly once when success is detected.
    // This is the "finish checker" bridge: checker -> success -> pause -> export.
    if (this.successExported) return;
    if (!this.onTrajectoryExport) return;
    if (!this.model || !this.data) return;
    if (this.replayPlaying) return;

    let checkerSuccessNow = false;
    try {
      if (this.checkerManager) {
        checkerSuccessNow = !!this.checkerManager.check();
      }
    } catch (e) {
      // ignore (status broadcasting will surface error)
    }

    if (!this.successAchieved && !checkerSuccessNow) return;

    // Mark exported first to avoid any re-entrancy / repeated calls.
    this.successExported = true;

    // Pause simulation so the UI time stops and state is stable.
    this.params.paused = true;
    if (this.guiPauseController) {
      this.guiPauseController.setValue(true);
    }

    // Ensure we have at least one final sample at the moment of completion.
    try {
      this.recordEpisodeSample(timeMS, true);
    } catch (e) {
      console.warn('[Teleop] Failed to record final episode sample on success:', e);
    }

    try {
      const serializedTrajectory = this.serializeTrajectory();
      const metadata = this.buildEpisodeMetadata(this.currentEpisode);
      this.onTrajectoryExport(serializedTrajectory, metadata);
    } catch (e) {
      console.error('[Teleop] Failed to export trajectory on success:', e);
    }

    // Push an immediate status update (will reflect paused + success).
    this.broadcastStatus();
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
      const frankaJointNames = [
        'franka/panda_joint1', 'franka/panda_joint2', 'franka/panda_joint3',
        'franka/panda_joint4', 'franka/panda_joint5', 'franka/panda_joint6',
        'franka/panda_joint7', 'franka/panda_finger_joint1', 'franka/panda_finger_joint2'
      ];
      for (const jointName of frankaJointNames) {
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
    if (this.episodeStartTime === null) {
      this.episodeStartTime = timeMS;
      this.episodeStartWallClock = Date.now();
      this.lastEpisodeSample = timeMS - this.episodeSampleIntervalMs;
      this.episodeStepCount = 0;
    }
    if (!force) {
      if (timeMS - this.lastEpisodeSample < this.episodeSampleIntervalMs) { return; }
      this.episodeStepCount++;
      
      // Check success conditions every step (not just when printing)
      if (!this.successAchieved) {
        try {
          // Initialize success timer if not started
          if (this.successStartTime === null) {
            this.resetSuccessTimer();
          }

          // Prefer dynamic checker config from backend (CheckerManager)
          let isSuccessNow = false;
          if (this.checkerManager) {
            isSuccessNow = !!this.checkerManager.check();
          } else {
            // Legacy fallback (should be unused once tasks provide checker_config)
            let bowlInRange = false;
            if (this.bowlPositionChecker) {
              const minBounds = new THREE.Vector3(0.3, 0.14, 0.12);
              const maxBounds = new THREE.Vector3(0.7, 0.22, 0.2);
              bowlInRange = this.bowlPositionChecker.checkInBounds(minBounds, maxBounds);
            }
            let drawerOpen = false;
            if (this.drawerPositionChecker) {
              const drawerStatus = this.drawerPositionChecker.getStatus();
              drawerOpen = drawerStatus.success;
            }
            isSuccessNow = bowlInRange && drawerOpen;
          }

          if (isSuccessNow) {
            this.successAchieved = true;
            this.successTime = performance.now() - this.successStartTime;
            const successTimeSeconds = (this.successTime / 1000).toFixed(2);
            console.log(`🎉🎉🎉 SUCCESS ACHIEVED! Time: ${successTimeSeconds}s (${this.successTime.toFixed(0)}ms) 🎉🎉🎉`);
          }
        } catch (error) {
          console.error(`[Step ${this.episodeStepCount}] Failed to check success conditions:`, error);
        }
      }

      // Print per-subgoal progress for current task every N steps (only while not yet success).
      if (!this.successAchieved && this.checkerManager && this.checkerProgressPrintEverySteps > 0) {
        if (this.episodeStepCount % this.checkerProgressPrintEverySteps === 0) {
          try {
            const report = this.checkerManager.getProgressReport?.();
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
              const line = `[Step ${this.episodeStepCount}] Goals: ${parts.join(' | ')}`;
              // Avoid spamming identical lines (but still prints periodically when progress changes).
              if (line !== this._lastCheckerProgressLine) {
                console.log(line);
                this._lastCheckerProgressLine = line;
              }
            }
          } catch (e) {
            console.warn('[Checker] Failed to print progress report:', e);
          }
        }
      }
      
      // Print bowl position and drawer dof pos every 20 steps
      if (this.episodeStepCount % 20 === 0) {
        try {
          // Print bowl position
          if (this.bowlPositionChecker) {
            const bowlStatus = this.bowlPositionChecker.getStatus();
            console.log(`[Step ${this.episodeStepCount}] Bowl position: (${bowlStatus.position.x.toFixed(4)}, ${bowlStatus.position.y.toFixed(4)}, ${bowlStatus.position.z.toFixed(4)})`);
            
            // Check if bowl is in success range [0.3, 0.14, 0.12] to [0.7, 0.22, 0.2]
            const minBounds = new THREE.Vector3(0.3, 0.14, 0.12);
            const maxBounds = new THREE.Vector3(0.7, 0.22, 0.2);
            const bowlInRange = this.bowlPositionChecker.checkInBounds(minBounds, maxBounds);
            if (bowlInRange) {
              console.log(`[Step ${this.episodeStepCount}] ✅ Bowl is in target range!`);
            }
          }
          
          // Print drawer joint positions (dof pos)
          if (this.drawerPositionChecker) {
            const drawerPositions = this.drawerPositionChecker.getDrawerPositions();
            const drawerPosStr = Object.entries(drawerPositions)
              .map(([name, pos]) => `${name}: ${pos.toFixed(4)}`)
              .join(', ');
            console.log(`[Step ${this.episodeStepCount}] Drawer dof pos: ${drawerPosStr}`);
            
            // Check if drawer is open (position < -0.1)
            const drawerStatus = this.drawerPositionChecker.getStatus();
            if (drawerStatus.success) {
              console.log(`[Step ${this.episodeStepCount}] ✅ Drawer is open! (position: ${drawerStatus.minPosition.toFixed(4)} < -0.1)`);
            }
          }
        } catch (error) {
          console.error(`[Step ${this.episodeStepCount}] Failed to get positions:`, error);
        }
      }

      // Print box dof pos every 30 steps (task4 close_box)
      if (this.episodeStepCount % 30 === 0) {
        try {
          const boxJointName = 'box_base/box_joint';
          const addr = this.getJointAddress(boxJointName);
          if (addr >= 0 && addr < this.data.qpos.length) {
            console.log(`[Step ${this.episodeStepCount}] Box dof pos: ${boxJointName}: ${this.data.qpos[addr].toFixed(4)}`);
          }
        } catch (error) {
          console.error(`[Step ${this.episodeStepCount}] Failed to get box dof pos:`, error);
        }
      }
      
      if (this.episodeStepCount % this.decimation !== 0) {
        return;
      }
    }
    this.lastEpisodeSample = timeMS;
    const sample = {
      timestamp: timeMS,
      elapsedMs: timeMS - this.episodeStartTime,
      step: this.episodeStepCount,
      qpos: Array.from(this.data.qpos),
      qvel: Array.from(this.data.qvel),
    };

    if (this.data.ctrl?.length) {
      sample.ctrl = Array.from(this.data.ctrl);
    }

    if (this.model.nmocap > 0) {
      sample.mocap_pos = Array.from(this.data.mocap_pos);
      sample.mocap_quat = Array.from(this.data.mocap_quat);
    }

    this.currentEpisode.push(sample);
  }

  buildEpisodeMetadata(samples) {
    const durationMs = samples.length ? samples[samples.length - 1].elapsedMs : 0;
    let averageIntervalMs = null;
    if (samples.length > 1) {
      let accumulator = 0;
      for (let i = 1; i < samples.length; i++) {
        accumulator += samples[i].elapsedMs - samples[i - 1].elapsedMs;
      }
      averageIntervalMs = accumulator / (samples.length - 1);
    }

    return {
      id: `trajectory-${Date.now()}`,
      scene: this.params.scene,
      sampleCount: samples.length,
      startedAt: this.episodeStartWallClock ? new Date(this.episodeStartWallClock).toISOString() : null,
      durationMs,
      desiredIntervalMs: this.episodeSampleIntervalMs,
      decimation: this.decimation,
      averageIntervalMs,
      model: {
        nq: this.model.nq,
        nv: this.model.nv,
        nu: this.model.nu,
        nbody: this.model.nbody,
        njnt: this.model.njnt,
      },
      control: this.robotControlConfig,
    };
  }

  async saveTrajectory(payload) {
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
      this.recordEpisodeSample(timeMS);
      // If the checker has transitioned to success, auto-export the trajectory once.
      this.maybeFinalizeSuccess(timeMS);
    } else if (this.params["paused"]) {
      this.mujoco.mj_forward(this.model, this.data);
      if (this.keyboardStateManager?.hasActiveCommands() || this.currentEpisode.length || this.ikController?.targetDirty) {
        this.recordEpisodeSample(timeMS);
      }
    }
    this.updateSceneFromData();
    this.updateCheckers(timeMS);
    this.renderer.render( this.scene, this.camera );
  }
  
  updateCheckers(timeMS) {
    if (timeMS - this.lastCheckerUpdate < this.checkerUpdateInterval) {
      return;
    }
    this.lastCheckerUpdate = timeMS;
    if (!this.checkerManager) return;
    try {
      // Poll status periodically (also useful for updating internal caches)
      this.checkerManager.getStatus();
    } catch (error) {
      console.error('[Checker] CheckerManager error:', error);
    }
  }
  
  getCheckerStatuses() {
    if (!this.checkerManager) return {};
    try {
      return this.checkerManager.getStatus();
    } catch (error) {
      return { error: error.message };
    }
  }

  // ========== React Integration Methods ==========
  
  /**
   * Broadcast current status to React parent via callback
   * Similar to HTML bridge's sendStatus() function
   */
  broadcastStatus() {
    if (!this.onStatusUpdate) return;
    
    try {
      // Count completed episodes + current episode if it has samples
      const completedEpisodes = this.episodes ? this.episodes.length : 0;
      const currentEpisode = this.currentEpisode || [];
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
      if (this.episodeStartTime !== null && this.episodeStartTime !== undefined) {
        if (currentEpisode.length > 0) {
          simulationElapsedMs = currentDurationMs;
        } else {
          const currentTime = performance.now();
          simulationElapsedMs = currentTime - this.episodeStartTime;
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
        checkerStatus = this.getCheckerStatuses();
        if (this.checkerManager) {
          checkerSuccess = !!this.checkerManager.check();
        }
      } catch (e) {
        checkerStatus = { error: e?.message || String(e) };
      }
      
      const status = {
        time: simulationElapsedMs,
        trajectoryCount: completedEpisodes + currentEpisodeActive,
        isSuccess: this.successAchieved || checkerSuccess || false,
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
      
      this.onStatusUpdate(status);
    } catch (e) {
      console.error("[MuJoCoDemo] Error broadcasting status:", e);
    }
  }

  /**
   * Start broadcasting status updates periodically
   */
  startStatusBroadcasting() {
    if (this.statusBroadcastInterval) {
      clearInterval(this.statusBroadcastInterval);
    }
    // Broadcast status every 100ms (10 updates per second)
    this.statusBroadcastInterval = setInterval(() => {
      this.broadcastStatus();
    }, 100);
  }

  /**
   * Stop broadcasting status updates
   */
  stopStatusBroadcasting() {
    if (this.statusBroadcastInterval) {
      clearInterval(this.statusBroadcastInterval);
      this.statusBroadcastInterval = null;
    }
  }

  /**
   * Serialize current episode trajectory to API format
   * Similar to HTML bridge's serializeTrajectory() function
   */
  serializeTrajectory() {
    if (!this.currentEpisode || this.currentEpisode.length === 0) {
      return [];
    }

    const robotJoints = this.robotControlConfig?.jointNames || [];
    const jointAddresses = {};
    
    // Cache joint addresses
    robotJoints.forEach(name => {
      const addr = this.getJointAddress(name);
      if (addr >= 0) jointAddresses[name] = addr;
    });

    // Get episode start time (Unix timestamp in milliseconds)
    const episodeStartUnix = this.episodeStartWallClock || Date.now();

    return this.currentEpisode.map(sample => {
      // Map robot joints
      const currentRobotJoints = {};
      robotJoints.forEach(name => {
        const addr = jointAddresses[name];
        if (addr !== undefined && sample.qpos && addr < sample.qpos.length) {
          currentRobotJoints[name] = sample.qpos[addr];
        }
      });

      // Convert to Unix timestamp: episodeStartUnix + elapsedMs, rounded to integer
      const unixTimestamp = Math.round(episodeStartUnix + sample.elapsedMs);

      return {
        timestamp: unixTimestamp, // Integer Unix timestamp in milliseconds
        simulation_time: sample.elapsedMs / 1000,
        state: {
          robot_joints: currentRobotJoints,
          robot_velocities: {}, // TODO: Map velocities if needed
          object_positions: {}, // TODO: Map object positions if needed
          object_velocities: {}
        },
        action: {
          controls: {} // TODO: Map controls if available
        }
      };
    });
  }

  /**
   * Build episode metadata for trajectory export
   */
  buildEpisodeMetadata(episode) {
    if (!episode || episode.length === 0) {
      return {
        sampleCount: 0,
        durationMs: 0,
        desiredIntervalMs: this.episodeSampleIntervalMs || 50,
      };
    }

    const lastSample = episode[episode.length - 1];
    return {
      sampleCount: episode.length,
      durationMs: lastSample.elapsedMs || 0,
      desiredIntervalMs: this.episodeSampleIntervalMs || 50,
    };
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

  /**
   * Public method: Trigger fake completion and export trajectory
   * Similar to HTML bridge's fake-done command
   */
  fakeDone() {
    console.log("[MuJoCoDemo] Fake-done command received");
    
    // 1. Pause simulation (stops time counting)
    this.params.paused = true;
    if (this.guiPauseController) {
      this.guiPauseController.setValue(true);
    }
    
    // 2. Trigger fake success
    this.successAchieved = true;
    this.successTime = performance.now() - (this.successStartTime || performance.now());
    
    // 3. Force final sample recording (captures current state)
    if (this.recordEpisodeSample) {
      this.recordEpisodeSample(performance.now(), true);
    }
    
    console.log("🎉 [FAKE] Task completed! Simulation paused.");
    console.log(`📊 Recorded ${this.currentEpisode?.length || 0} samples`);
    
    // 4. Serialize and export trajectory
    if (this.onTrajectoryExport) {
      const serializedTrajectory = this.serializeTrajectory();
      const metadata = this.buildEpisodeMetadata(this.currentEpisode);
      this.onTrajectoryExport(serializedTrajectory, metadata);
    }
    
    // Send immediate status update
    this.broadcastStatus();
  }

  /**
   * Public method: Load trajectory for replay
   * @param {Array} samples - Trajectory samples to replay
   */
  loadTrajectory(samples) {
    if (!samples || !Array.isArray(samples) || samples.length === 0) {
      console.error("[MuJoCoDemo] Invalid trajectory samples provided");
      return;
    }
    
    console.log(`[MuJoCoDemo] Loading trajectory with ${samples.length} samples for replay`);
    
    // Store samples and flag for replay
    this.pendingReplaySamples = samples;
    this.pendingReplayFlag = true;
    
    // Do the same reset sequence as reset() command
    this.clearEpisodeState();
    this.successAchieved = false;
    this.successTime = null;
    this.resetSuccessTimer();
    this.resetSimulation();
    this.params.paused = false;
    if (this.guiPauseController) {
      this.guiPauseController.setValue(false);
    }
    
    // Load and start replay
    this.loadTrajectoryFromData(samples);
    
    console.log(`✅ [MuJoCoDemo] Reset complete, loaded ${samples.length} trajectory samples for replay`);
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
