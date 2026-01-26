/**
 * BoxJointPositionChecker
 * Checks if the box joint position meets a threshold condition (success condition)
 * For close_box task: checks if box lid is closed (position < threshold)
 * 
 * Supports two config formats:
 * 1. Legacy: { threshold, jointName }
 * 2. Task config: { target_position, tolerance, joint_name, comparison }
 */
export class BoxJointPositionChecker {
  constructor(mujoco, model, data, options = {}) {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;
    
    // Support both camelCase and snake_case config formats
    this.jointName = options.jointName || options.joint_name || 'box_base/box_joint';
    
    // Support task config format: target_position + tolerance
    if (options.target_position !== undefined) {
      const target = Number(options.target_position);
      const tolerance = Number(options.tolerance ?? 0.1);
      this.threshold = target + tolerance;
    } else {
      // Legacy format: direct threshold
      // Default threshold: -14 degrees = -14 * π / 180 ≈ -0.244346 radians
      this.threshold = options.threshold ?? (-14 / 180 * Math.PI);
    }
    
    // Comparison mode: 'less_than' (default) or 'greater_than'
    this.comparison = options.comparison || 'less_than';
    
    this.textDecoder = new TextDecoder('utf-8');
    this.namesBuffer = new Uint8Array(model.names);
    
    // Resolve joint address
    this.jointAddress = this.getJointAddress(this.jointName);
    
    if (this.jointAddress < 0) {
      console.warn(`[BoxJointPositionChecker] Joint "${this.jointName}" not found`);
    } else {
      console.log(`[BoxJointPositionChecker] Initialized with joint "${this.jointName}", threshold=${this.threshold.toFixed(6)} radians, comparison="${this.comparison}"`);
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
   * Check if box joint position meets threshold condition (success condition)
   * @returns {boolean} True if condition met, false otherwise
   */
  check() {
    if (this.jointAddress < 0) {
      return false;
    }
    
    if (this.jointAddress >= this.data.qpos.length) {
      return false;
    }
    
    const position = this.data.qpos[this.jointAddress];
    
    if (this.comparison === 'greater_than') {
      return position > this.threshold;
    }
    // Default: less_than
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
    const isSuccess = this.check();
    const compOp = this.comparison === 'greater_than' ? '>' : '<';
    const compOpNot = this.comparison === 'greater_than' ? '<=' : '>=';
    
    return {
      success: isSuccess,
      threshold: this.threshold,
      position: position,
      comparison: this.comparison,
      jointName: this.jointName,
      message: isSuccess 
        ? `Success! (position: ${position !== null ? position.toFixed(4) : 'N/A'} ${compOp} ${this.threshold.toFixed(4)})`
        : `Not yet (position: ${position !== null ? position.toFixed(4) : 'N/A'} ${compOpNot} ${this.threshold.toFixed(4)})`
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






