import * as THREE from 'three';

const DEFAULT_MOVEMENT_BINDINGS = {
  ArrowUp:   new THREE.Vector3(1, 0, 0),   // +X forward
  ArrowDown: new THREE.Vector3(-1, 0, 0),  // -X backward
  ArrowLeft: new THREE.Vector3(0, 1, 0),   // +Y left
  ArrowRight:new THREE.Vector3(0, -1, 0),  // -Y right
  KeyE:      new THREE.Vector3(0, 0, 1),   // +Z up
  KeyD:      new THREE.Vector3(0, 0, -1),  // -Z down
};

const DEFAULT_ROTATION_BINDINGS = {
  KeyQ: new THREE.Vector3(1, 0, 0),   // roll +
  KeyW: new THREE.Vector3(-1, 0, 0),  // roll -
  KeyA: new THREE.Vector3(0, 1, 0),   // pitch +
  KeyS: new THREE.Vector3(0, -1, 0),  // pitch -
  KeyZ: new THREE.Vector3(0, 0, 1),   // yaw +
  KeyX: new THREE.Vector3(0, 0, -1),  // yaw -
};

const GRIPPER_KEY = 'Space';
const CHECKPOINT_SAVE_KEY = 'KeyN';      // N键：存档（save checkpoint）
const CHECKPOINT_RESTORE_KEY = 'KeyB';   // B键：回到上个存档点（restore checkpoint）
const REPLAY_KEY = 'KeyP';
const TOGGLE_KEYS = new Set(['KeyV', 'KeyR', 'Escape']);

export class KeyboardStateManager {
  constructor(options = {}) {
    // Base speeds (dpos/drot)
    this.translationSpeed = options.translationSpeed ?? 0.25; // meters per second
    this.rotationSpeed = options.rotationSpeed ?? THREE.MathUtils.degToRad(45); // rad/sec

    // Modifiers: hold Shift to move/rotate faster; hold Alt to move/rotate slower.
    this.fastMultiplier = options.fastMultiplier ?? 3.0;
    this.slowMultiplier = options.slowMultiplier ?? 0.35;
    this.shiftDown = false;
    this.altDown = false;
    this.movementBindings = options.movementBindings ?? DEFAULT_MOVEMENT_BINDINGS;
    this.rotationBindings = options.rotationBindings ?? DEFAULT_ROTATION_BINDINGS;

    this.onTranslate = options.onTranslate;
    this.onRotate = options.onRotate;
    this.onGripperCommand = options.onGripperCommand;
    this.onCompleteEpisode = options.onCompleteEpisode;
    this.onResetEpisode = options.onResetEpisode;
    this.onSaveAndExit = options.onSaveAndExit;
    this.onSaveCheckpoint = options.onSaveCheckpoint;
    this.onRestoreCheckpoint = options.onRestoreCheckpoint;
    this.onReplayTrajectory = options.onReplayTrajectory;

    // Gripper key behavior:
    // - 'hold': keydown => close, keyup => open (old behavior)
    // - 'toggle': each key press toggles open/close once (no need to hold)
    this.gripperMode = options.gripperMode ?? 'hold';

    this.activeKeys = new Set();
    this.pressedKeys = new Set();
    this.lastTimestamp = null;

    this._handleKeyDown = this.handleKeyDown.bind(this);
    this._handleKeyUp = this.handleKeyUp.bind(this);

    document.addEventListener('keydown', this._handleKeyDown);
    document.addEventListener('keyup', this._handleKeyUp);
  }

  dispose() {
    document.removeEventListener('keydown', this._handleKeyDown);
    document.removeEventListener('keyup', this._handleKeyUp);
  }

  handleKeyDown(event) {
    if (!event.repeat) {
      this.pressedKeys.add(event.code);
    }
    if (event.code === 'ShiftLeft' || event.code === 'ShiftRight') {
      this.shiftDown = true;
    }
    if (event.code === 'AltLeft' || event.code === 'AltRight') {
      this.altDown = true;
    }

    const isMovementKey = Boolean(this.movementBindings[event.code]);
    const isRotationKey = Boolean(this.rotationBindings[event.code]);
    if (isMovementKey || isRotationKey || event.code === GRIPPER_KEY ||
        TOGGLE_KEYS.has(event.code) || event.code === CHECKPOINT_SAVE_KEY ||
        event.code === CHECKPOINT_RESTORE_KEY) {
      event.preventDefault();
    }

    if (event.code === GRIPPER_KEY) {
      // Ignore key auto-repeat for toggle mode (holding space shouldn't spam toggles)
      if (this.gripperMode === 'toggle') {
        if (event.repeat) { return; }
        if (this.onGripperCommand) { this.onGripperCommand(); }
      } else {
        this.handleGripper(true);
      }
      return;
    }

    if (event.code === CHECKPOINT_SAVE_KEY) {
      if (this.onSaveCheckpoint) { this.onSaveCheckpoint(); }
      return;
    }

    if (event.code === CHECKPOINT_RESTORE_KEY) {
      if (this.onRestoreCheckpoint) { this.onRestoreCheckpoint(); }
      return;
    }

    if (event.code === REPLAY_KEY) {
      this.handleReplay();
      return;
    }

    if (TOGGLE_KEYS.has(event.code)) {
      this.handleToggle(event.code);
      return;
    }

    if (isMovementKey || isRotationKey) {
      this.activeKeys.add(event.code);
    }
  }

  handleKeyUp(event) {
    if (event.code === 'ShiftLeft' || event.code === 'ShiftRight') {
      this.shiftDown = false;
    }
    if (event.code === 'AltLeft' || event.code === 'AltRight') {
      this.altDown = false;
    }
    if (event.code === GRIPPER_KEY) {
      // In toggle mode, keyup does nothing.
      if (this.gripperMode !== 'toggle') {
        this.handleGripper(false);
      }
      return;
    }
    this.activeKeys.delete(event.code);
  }

  handleToggle(code) {
    switch (code) {
      case 'KeyV':
        if (this.onCompleteEpisode) { this.onCompleteEpisode(); }
        break;
      case 'KeyR':
        if (this.onResetEpisode) { this.onResetEpisode(); }
        break;
      case 'Escape':
        if (this.onSaveAndExit) { this.onSaveAndExit(); }
        break;
      default:
        break;
    }
  }

  handleReplay() {
    if (this.onReplayTrajectory) {
      this.onReplayTrajectory();
    }
  }

  handleGripper(state) {
    if (this.onGripperCommand) {
      this.onGripperCommand(state);
    }
  }

  update(timestampMS) {
    if (this.lastTimestamp === null) {
      this.lastTimestamp = timestampMS;
      return;
    }

    const deltaS = Math.min((timestampMS - this.lastTimestamp) / 1000.0, 0.1);
    this.lastTimestamp = timestampMS;

    if (deltaS <= 0) { return; }

    const speedScale = this.shiftDown ? this.fastMultiplier : (this.altDown ? this.slowMultiplier : 1.0);

    const translation = new THREE.Vector3();
    const activeMovementKeys = [];
    for (const [code, direction] of Object.entries(this.movementBindings)) {
      if (this.activeKeys.has(code)) {
        translation.add(direction);
        activeMovementKeys.push(code);
      }
    }

    if (translation.lengthSq() > 0) {
      if (this.onTranslate) {
        translation.normalize().multiplyScalar(this.translationSpeed * speedScale * deltaS);
        this.onTranslate(translation);
      } else {
        if (!this._translateWarningShown) {
          console.warn('[KeyboardStateManager] onTranslate callback not set!');
          this._translateWarningShown = true;
        }
      }
    }

    const rotation = new THREE.Vector3();
    const activeRotationKeys = [];
    for (const [code, axis] of Object.entries(this.rotationBindings)) {
      if (this.activeKeys.has(code)) {
        rotation.add(axis);
        activeRotationKeys.push(code);
      }
    }

    if (rotation.lengthSq() > 0) {
      if (this.onRotate) {
        rotation.multiplyScalar(this.rotationSpeed * speedScale * deltaS);
        this.onRotate(rotation);
      } else {
        if (!this._rotateWarningShown) {
          console.warn('[KeyboardStateManager] onRotate callback not set!');
          this._rotateWarningShown = true;
        }
      }
    }
  }

  hasActiveCommands() {
    return this.activeKeys.size > 0;
  }

  clearActiveKeys() {
    this.activeKeys.clear();
  }

  clearPressedKeys() {
    this.pressedKeys.clear();
  }

  getPressedKeys() {
    return this.pressedKeys;
  }
}
