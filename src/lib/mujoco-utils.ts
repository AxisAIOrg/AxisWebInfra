/**
 * Shared MuJoCo utilities used by both MuJoCoViewer component and mujoco test page
 */

// Declare global type for mujoco module loaded via script tag
declare global {
  interface Window {
    load_mujoco?: () => Promise<any>;
  }
}

/**
 * Load MuJoCo module using script tag (bypasses webpack completely)
 * This is the working strategy that loads MuJoCo WASM at runtime
 */
export async function loadMujoco() {
  if (typeof window === "undefined") {
    throw new Error("MuJoCo can only be loaded in the browser");
  }
  
  // Strategy: Load via script tag (completely bypasses webpack)
  const loadViaScript = (): Promise<any> => {
    return new Promise((resolve, reject) => {
      // Check if already loaded
      if (window.load_mujoco) {
        resolve(window.load_mujoco);
        return;
      }
      
      // Remove any existing script
      const existingScript = document.getElementById("mujoco-wasm-script");
      if (existingScript) {
        existingScript.remove();
      }
      
      // Create script tag
      const script = document.createElement("script");
      script.id = "mujoco-wasm-script";
      script.type = "module";
      script.src = "/mujoco-js/dist/mujoco_wasm.js";
      
      script.onload = () => {
        // Import the module after script loads (it should be cached)
        setTimeout(async () => {
          try {
            const importModule = new Function(
              'return import("/mujoco-js/dist/mujoco_wasm.js")'
            ) as () => Promise<any>;
            
            const module = await importModule();
            const loader = module.default || module;
            window.load_mujoco = loader;
            resolve(loader);
          } catch (importError: any) {
            reject(importError);
          }
        }, 100);
      };
      
      script.onerror = () => {
        reject(new Error("Failed to load MuJoCo script"));
      };
      
      document.head.appendChild(script);
    });
  };
  
  try {
    const load_mujoco = await loadViaScript();
    
    if (!load_mujoco || typeof load_mujoco !== "function") {
      throw new Error("MuJoCo loader is not a function");
    }
    
    const mujocoInstance = await load_mujoco();
    
    if (!mujocoInstance) {
      throw new Error("MuJoCo instance is null or undefined");
    }
    
    return mujocoInstance;
  } catch (error) {
    console.error("[MuJoCo Utils] Failed to load MuJoCo module:", error);
    throw error;
  }
}

/**
 * Helper function to convert MuJoCo position buffer to Three.js coordinates
 * MuJoCo uses different coordinate system than Three.js (Y and Z are swapped)
 */
export function getPos(
  buffer: Float32Array | Float64Array,
  index: number,
): [number, number, number] {
  return [
    buffer[index * 3 + 0],
    buffer[index * 3 + 2], // Y and Z swapped
    -buffer[index * 3 + 1],
  ];
}

/**
 * Helper function to convert MuJoCo quaternion buffer to Three.js quaternion
 * MuJoCo uses different quaternion convention than Three.js
 */
export function getQuat(
  buffer: Float32Array | Float64Array,
  index: number,
): [number, number, number, number] {
  return [
    -buffer[index * 4 + 1],
    -buffer[index * 4 + 3],
    buffer[index * 4 + 2],
    -buffer[index * 4 + 0],
  ];
}


