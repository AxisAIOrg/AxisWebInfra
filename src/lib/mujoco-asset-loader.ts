/**
 * Utility functions for loading MuJoCo assets (textures, meshes) into the virtual file system
 */
import { unzipSync } from "fflate";

type AssetZipIndex = Map<string, Uint8Array>;

const zipIndexCache = new Map<string, AssetZipIndex>();

function normalizeZipPath(path: string): string {
  const raw = path.replace(/\\/g, "/").replace(/^\.?\//, "");
  const parts = raw.split("/").filter((part) => part.length > 0);
  const stack: string[] = [];
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

function buildZipCandidates(assetPath: string): string[] {
  const normalized = normalizeZipPath(assetPath);
  const withoutUnderscorePrefix = normalized.startsWith("mujoco_assets/")
    ? normalized.slice("mujoco_assets/".length)
    : normalized;
  const withoutHyphenPrefix = normalized.startsWith("mujoco-assets/")
    ? normalized.slice("mujoco-assets/".length)
    : normalized;
  const candidates = new Set<string>();

  candidates.add(normalized);
  candidates.add(withoutUnderscorePrefix);
  candidates.add(withoutHyphenPrefix);
  candidates.add(`mujoco-assets/${withoutUnderscorePrefix}`);
  candidates.add(`mujoco_assets/${withoutUnderscorePrefix}`);

  return Array.from(candidates).filter(Boolean);
}

async function getZipIndex(zipUrl: string): Promise<AssetZipIndex> {
  const cached = zipIndexCache.get(zipUrl);
  if (cached) {
    return cached;
  }

  const response = await fetch(zipUrl);
  if (!response.ok) {
    throw new Error(`Failed to fetch asset zip ${zipUrl}: ${response.statusText}`);
  }

  const arrayBuffer = await response.arrayBuffer();
  const entries = unzipSync(new Uint8Array(arrayBuffer));
  const index = new Map<string, Uint8Array>();

  for (const [name, data] of Object.entries(entries)) {
    index.set(normalizeZipPath(name), data);
  }

  zipIndexCache.set(zipUrl, index);
  return index;
}

/**
 * Load a file from a URL and write it to MuJoCo's virtual file system
 */
export async function loadAssetToVFS(
  mujoco: any,
  url: string,
  vfsPath: string
): Promise<void> {
  try {
    const response = await fetch(url);
    if (!response.ok) {
      throw new Error(`Failed to fetch ${url}: ${response.statusText}`);
    }
    
    // For binary files (meshes, images), use arrayBuffer
    const isBinary = url.endsWith('.msh') || url.endsWith('.obj') || url.endsWith('.stl') || url.endsWith('.png') || url.endsWith('.jpg');
    
    if (isBinary) {
      const arrayBuffer = await response.arrayBuffer();
      const uint8Array = new Uint8Array(arrayBuffer);
      mujoco.FS.writeFile(vfsPath, uint8Array);
    } else {
      // For text files
      const text = await response.text();
      mujoco.FS.writeFile(vfsPath, text);
    }
  } catch (error) {
    console.error(`Error loading asset ${url} to ${vfsPath}:`, error);
    throw error;
  }
}

/**
 * Parse XML to extract asset file paths (textures and meshes)
 */
export function extractAssetPaths(xml: string): { textures: string[]; meshes: string[] } {
  const textures: string[] = [];
  const meshes: string[] = [];
  
  // Extract texture file paths
  const textureRegex = /<texture[^>]*file=["']([^"']+)["'][^>]*>/gi;
  let match;
  while ((match = textureRegex.exec(xml)) !== null) {
    textures.push(match[1]);
  }
  
  // Extract mesh file paths
  const meshRegex = /<mesh[^>]*file=["']([^"']+)["'][^>]*>/gi;
  while ((match = meshRegex.exec(xml)) !== null) {
    meshes.push(match[1]);
  }
  
  return { textures, meshes };
}

/**
 * Load all assets referenced in the XML into the virtual file system
 * Assets are loaded from the frontend public directory /mujoco-assets/
 */
export async function loadMuJoCoAssets(
  mujoco: any,
  xml: string,
  baseUrlOrOptions?: string | { baseUrl?: string; zipUrl?: string; zipOnly?: boolean }
): Promise<void> {
  // Use API base URL if not provided
  let apiBase = `/mujoco-assets/`;
  let zipUrl: string | undefined;
  let zipOnly = false;
  if (typeof baseUrlOrOptions === "string") {
    apiBase = baseUrlOrOptions;
  } else if (baseUrlOrOptions) {
    apiBase = baseUrlOrOptions.baseUrl || apiBase;
    zipUrl = baseUrlOrOptions.zipUrl;
    zipOnly = Boolean(baseUrlOrOptions.zipOnly);
  }
  const { textures, meshes } = extractAssetPaths(xml);
  const allAssets = [...textures, ...meshes];
  
  // Remove duplicates
  const uniqueAssets = Array.from(new Set(allAssets));

  let zipIndex: AssetZipIndex | null = null;
  if (zipUrl) {
    try {
      zipIndex = await getZipIndex(zipUrl);
    } catch (error) {
      console.warn(`⚠ Failed to load asset zip ${zipUrl}, falling back to individual fetches.`, error);
    }
  }
  if (zipOnly && !zipIndex) {
    throw new Error(`zipOnly is enabled but asset zip could not be loaded: ${zipUrl || "(missing url)"}`);
  }
  
  // Create necessary directories and load each asset
  for (const assetPath of uniqueAssets) {
    // Strip directory prefix from API path (backend already knows this directory)
    // But keep it for VFS path (MuJoCo needs the full path as specified in XML)
    let apiPath = assetPath;
    if (assetPath.startsWith('mujoco_assets/')) {
      apiPath = assetPath.substring('mujoco_assets/'.length);
    }
    
    // Create directory structure in VFS
    const pathParts = assetPath.split('/');
    const fileName = pathParts.pop()!;
    const dirPath = pathParts.join('/');
    
    if (dirPath) {
      // Create directory path in VFS (relative to /working)
      const vfsDirPath = `/working/${dirPath}`;
      try {
        // Create parent directories recursively
        const parts = dirPath.split('/');
        let currentPath = '/working';
        for (const part of parts) {
          currentPath = `${currentPath}/${part}`;
          try {
            mujoco.FS.mkdir(currentPath);
          } catch (e: any) {
            if (!e.message?.includes('File exists')) {
              throw e;
            }
          }
        }
      } catch (e: any) {
        console.warn(`Could not create directory ${vfsDirPath}:`, e);
      }
    }
    
    // Load asset from backend (use apiPath without mujoco_assets/ prefix)
    const assetUrl = `${apiBase}/${apiPath}`;
    // VFS path keeps the full path as specified in XML
    const vfsPath = `/working/${assetPath}`;
    
    try {
      let loadedFromZip = false;
      if (zipIndex) {
        const candidates = buildZipCandidates(assetPath);
        for (const candidate of candidates) {
          const entry = zipIndex.get(candidate);
          if (entry) {
            mujoco.FS.writeFile(vfsPath, entry);
            loadedFromZip = true;
            break;
          }
        }
      }

      if (!loadedFromZip) {
        if (zipOnly) {
          throw new Error(`Asset not found in zip: ${assetPath}`);
        }
        await loadAssetToVFS(mujoco, assetUrl, vfsPath);
      }
      
      // Verify the file was written to VFS
      try {
        const stats = mujoco.FS.stat(vfsPath);
        // console.log(`✓ Loaded asset: ${assetPath} (API: ${apiPath}) -> VFS: ${vfsPath} (${stats.size} bytes)`);
      } catch (statError) {
        console.warn(`⚠ Asset loaded but cannot verify in VFS: ${vfsPath}`, statError);
      }
    } catch (error) {
      console.warn(`⚠ Failed to load asset ${assetPath} (API: ${apiPath}), continuing...`, error);
      // Continue loading other assets even if one fails
    }
  }
}
