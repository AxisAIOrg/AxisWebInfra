/**
 * BoxJointPositionChecker
 * Checks if the box joint position is less than a threshold (success condition)
 * For close_box task: checks if box lid is closed (position < -14 degrees in radians)
 */
export class BoxJointPositionChecker {
  constructor(mujoco, model, data, options = {}) {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;
    
    // Configuration
    // Default threshold: -14 degrees = -14 * π / 180 ≈ -0.244346 radians
    this.threshold = options.threshold ?? (-14 / 180 * Math.PI);
    this.jointName = options.jointName ?? 'box_base/box_joint';
    
    this.textDecoder = new TextDecoder('utf-8');
    this.namesBuffer = new Uint8Array(model.names);
    
    // Resolve joint address
    this.jointAddress = this.getJointAddress(this.jointName);
    
    if (this.jointAddress < 0) {
      console.warn(`[BoxJointPositionChecker] Joint "${this.jointName}" not found`);
    } else {
      console.log(`[BoxJointPositionChecker] Initialized with joint "${this.jointName}", threshold=${this.threshold.toFixed(6)} radians (${(this.threshold * 180 / Math.PI).toFixed(2)} degrees)`);
    }
  }
  
  getJointAddress(jointName) {
    for (let j = 0; j < this.model.njnt; j++) {
      const nameStart = this.model.name_jntadr[j];
      let nameEnd = nameStart;
      while (nameEnd < this.namesBuffer.length && this.namesBuffer[nameEnd] !== 0) {
        nameEnd++;
      }
      const name = this.textDecoder.decode(this.namesBuffer.subarray(nameStart, nameEnd));
      if (name === jointName) {
        return this.model.jnt_qposadr[j];
      }
    }
    return -1;
  }
  
  /**
   * Check if box joint position < threshold (success condition)
   * @returns {boolean} True if box joint position < threshold, false otherwise
   */
  check() {
    if (this.jointAddress < 0) {
      return false;
    }
    
    if (this.jointAddress >= this.data.qpos.length) {
      return false;
    }
    
    const position = this.data.qpos[this.jointAddress];
    return position < this.threshold;
  }
  
  /**
   * Get current box joint position
   * @returns {number|null} Current joint position in radians, or null if joint not found
   */
  getPosition() {
    if (this.jointAddress < 0 || this.jointAddress >= this.data.qpos.length) {
      return null;
    }
    return this.data.qpos[this.jointAddress];
  }
  
  /**
   * Get position in degrees
   * @returns {number|null} Current joint position in degrees, or null if joint not found
   */
  getPositionDegrees() {
    const rad = this.getPosition();
    return rad !== null ? rad * 180 / Math.PI : null;
  }
  
  /**
   * Get detailed status
   * @returns {Object} Status object with check result and details
   */
  getStatus() {
    const position = this.getPosition();
    const positionDegrees = this.getPositionDegrees();
    const isSuccess = this.check();
    const thresholdDegrees = this.threshold * 180 / Math.PI;
    
    return {
      success: isSuccess,
      threshold: this.threshold,
      thresholdDegrees: thresholdDegrees,
      position: position,
      positionDegrees: positionDegrees,
      jointName: this.jointName,
      message: isSuccess 
        ? `盒子已关闭 (位置: ${positionDegrees !== null ? positionDegrees.toFixed(2) : 'N/A'}° < ${thresholdDegrees.toFixed(2)}°)`
        : `盒子未关闭 (位置: ${positionDegrees !== null ? positionDegrees.toFixed(2) : 'N/A'}° >= ${thresholdDegrees.toFixed(2)}°)`
    };
  }
  
  /**
   * Update model/data when model reloads
   */
  updateModelData(model, data) {
    this.model = model;
    this.data = data;
    this.namesBuffer = new Uint8Array(model.names);
    // Re-resolve joint address in case model changed
    this.jointAddress = this.getJointAddress(this.jointName);
  }
}






