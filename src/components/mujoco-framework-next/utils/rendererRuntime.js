import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

function setupRenderer(demo, parentElement) {
  demo.scene = new THREE.Scene();
  demo.scene.name = 'scene';

  // Get container dimensions instead of window dimensions
  const containerWidth = demo.container.clientWidth || parentElement.clientWidth || window.innerWidth;
  const containerHeight = demo.container.clientHeight || parentElement.clientHeight || window.innerHeight;

  demo.camera = new THREE.PerspectiveCamera(45, containerWidth / containerHeight, 0.001, 100);
  demo.camera.name = 'PerspectiveCamera';
  demo.camera.position.set(2.0, 1.7, 1.7);
  demo.scene.add(demo.camera);

  demo.scene.background = new THREE.Color(0.15, 0.25, 0.35);
  demo.scene.fog = new THREE.Fog(demo.scene.background, 15, 25.5);

  demo.ambientLight = new THREE.AmbientLight(0xffffff, 0.1 * 3.14);
  demo.ambientLight.name = 'AmbientLight';
  demo.scene.add(demo.ambientLight);

  demo.spotlight = new THREE.SpotLight();
  demo.spotlight.angle = 1.11;
  demo.spotlight.distance = 10000;
  demo.spotlight.penumbra = 0.5;
  demo.spotlight.castShadow = true;
  demo.spotlight.intensity = demo.spotlight.intensity * 3.14 * 10.0;
  demo.spotlight.shadow.mapSize.width = 1024;
  demo.spotlight.shadow.mapSize.height = 1024;
  demo.spotlight.shadow.camera.near = 0.1;
  demo.spotlight.shadow.camera.far = 100;
  demo.spotlight.position.set(0, 3, 3);
  const targetObject = new THREE.Object3D();
  demo.scene.add(targetObject);
  demo.spotlight.target = targetObject;
  targetObject.position.set(0, 1, 0);
  demo.scene.add(demo.spotlight);

  demo.renderer = new THREE.WebGLRenderer({ antialias: true });
  demo.renderer.setPixelRatio(1.0);
  // Use container dimensions instead of window dimensions
  demo.renderer.setSize(containerWidth, containerHeight);
  demo.renderer.shadowMap.enabled = true;
  demo.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  THREE.ColorManagement.enabled = false;
  demo.renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
  demo.renderer.useLegacyLights = true;

  // Ensure canvas doesn't break out of container
  demo.renderer.domElement.style.display = 'block';
  demo.renderer.domElement.style.width = '100%';
  demo.renderer.domElement.style.height = '100%';
  demo.renderer.domElement.style.maxWidth = '100%';
  demo.renderer.domElement.style.maxHeight = '100%';

  demo.renderer.setAnimationLoop(demo.render.bind(demo));

  demo.container.appendChild(demo.renderer.domElement);

  demo.controls = new OrbitControls(demo.camera, demo.renderer.domElement);
  demo.controls.target.set(0, 0.7, 0);
  demo.controls.panSpeed = 2;
  demo.controls.zoomSpeed = 1;
  demo.controls.enableDamping = true;
  demo.controls.dampingFactor = 0.10;
  demo.controls.screenSpacePanning = true;
  demo.controls.update();

  // Use ResizeObserver to watch container size changes (more accurate than window resize)
  if (typeof ResizeObserver !== 'undefined') {
    demo.resizeObserver = new ResizeObserver(() => {
      demo.onWindowResize();
    });
    demo.resizeObserver.observe(demo.container);
  } else {
    // Fallback to window resize for older browsers
    window.addEventListener('resize', demo.onWindowResize.bind(demo));
  }
  // Drag interaction intentionally disabled.
  // demo.dragStateManager = new DragStateManager(...);
}

export { setupRenderer };
