import { loadSceneFromURL } from '../mujocoUtils.js';
import { preloadSceneWithAssets, parseDomainRandomizationFromXML } from './assetLoader.js';
import { loadMujoco } from '@/lib/mujoco-utils';

async function ensureMujocoReady(demo, initialScene) {
  if (demo.mujoco) {
    return demo.mujoco;
  }
  demo.emitLoadingProgress({
    phase: "mujoco",
    message: "Loading MuJoCo runtime…",
    percent: 5,
  });
  const mujoco = await loadMujoco();
  demo.mujoco = mujoco;
  try {
    mujoco.FS.mkdir('/working');
  } catch (e) {
    // directory may already exist
  }
  mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
  demo.emitLoadingProgress({
    phase: "assets",
    message: "Preloading base scene assets…",
    percent: 10,
  });
  await preloadSceneWithAssets(mujoco, initialScene, demo.emitLoadingProgress.bind(demo));
  return mujoco;
}

async function resolveSceneFileFromTask(demo, mujoco, initialScene) {
  // Determine which scene to load
  let sceneFileToLoad = initialScene;

  // 1. Check URL Params for task id (use "id" to align with page.tsx)
  const urlParams = new URLSearchParams(window.location.search);
  const taskId = urlParams.get('id');

  if (!taskId) {
    return sceneFileToLoad;
  }

  console.log(`[main] Found taskId=${taskId} in URL, fetching from API...`);
  try {
    // Use API base URL from window (injected by HTML) or fallback to /api
    const apiBase = window.MUJOCO_API_BASE || '/api';
    const apiUrl = `${apiBase}/tasks/${taskId}`;
    console.log(`[main] Fetching task from: ${apiUrl}`);
    demo.emitLoadingProgress({
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
      if (!demo.domainRandomizationConfig) {
        const drConfig = parseDomainRandomizationFromXML(taskData.mjcf_xml);
        if (drConfig) {
          demo.domainRandomizationConfig = drConfig;
        } else {
          demo.domainRandomizationConfig = null;
        }
      }
    } else {
      console.warn("[main] Task has no XML, falling back...");
    }
  } catch (error) {
    console.error("[main] Failed to fetch task XML:", error);
    console.warn("[main] Fallback reason: Failed to fetch task XML from API.");
  }

  return sceneFileToLoad;
}

async function loadSceneWithFallback(demo, mujoco, sceneFileToLoad, initialScene) {
  demo.params.scene = sceneFileToLoad;
  console.log(`[main] Loading scene file: ${sceneFileToLoad}`);
  const readXmlFromVfs = (filename) => {
    try {
      return mujoco.FS.readFile("/working/" + filename, { encoding: 'utf8' });
    } catch (error) {
      console.warn(`[main] Failed to read XML from VFS: ${filename}`, error);
      return null;
    }
  };
  // Preload the scene (and assets) we are about to load so VFS contains referenced files
  try {
    demo.emitLoadingProgress({
      phase: "assets",
      message: "Preloading task scene assets…",
      percent: 25,
    });
    await preloadSceneWithAssets(mujoco, sceneFileToLoad, demo.emitLoadingProgress.bind(demo));
  } catch (error) {
    console.warn(`[main] Failed to preload ${sceneFileToLoad}: ${error.message}`);
  }
  try {
    demo.emitLoadingProgress({
      phase: "model",
      message: "Parsing MuJoCo model…",
      percent: 90,
    });
    demo.modelXml = readXmlFromVfs(sceneFileToLoad);
    [demo.model, demo.data, demo.bodies, demo.lights] = await loadSceneFromURL(mujoco, sceneFileToLoad, demo);
  } catch (error) {
    console.error(`[main] Failed to load ${sceneFileToLoad}, trying default fallback...`);
    console.warn(`[main] Fallback reason: ${error.message}`);
    demo.emitLoadingProgress({
      phase: "model",
      message: "Parsing fallback MuJoCo model…",
      percent: 90,
    });
    demo.modelXml = readXmlFromVfs(initialScene);
    [demo.model, demo.data, demo.bodies, demo.lights] = await loadSceneFromURL(mujoco, initialScene, demo);
  }
}

export {
  ensureMujocoReady,
  resolveSceneFileFromTask,
  loadSceneWithFallback,
};
