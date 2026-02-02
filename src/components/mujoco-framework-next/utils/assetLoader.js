import { unzipSync } from "fflate";

const LOCAL_SCENE_BASE_URL = "/mujoco-assets/";
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
      const combinedPath = xmlDir ? `${xmlDir}/${normalizedRelative}` : normalizedRelative;
      const assetPath = normalizeZipPath(combinedPath);
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

export {
  normalizeZipPath,
  preloadSceneWithAssets,
  parseDomainRandomizationFromXML,
};
