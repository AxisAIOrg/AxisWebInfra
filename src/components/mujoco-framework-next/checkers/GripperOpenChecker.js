/**
 * GripperOpenChecker
 * Checks if the gripper (finger joints) is open (joint angle > threshold)
 */
export class GripperOpenChecker {
  constructor(mujoco, model, data, options = {}) {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;
    
    // Configuration
    this.threshold = options.threshold ?? 0.2; // Default threshold: 0.2 (20mm for Franka gripper)
    this.fingerJointNames = options.fingerJointNames ?? [
      'franka/panda_finger_joint1',
      'franka/panda_finger_joint2'
    ];
    
    this.textDecoder = new TextDecoder('utf-8');
    this.namesBuffer = new Uint8Array(model.names);
    
    // Resolve joint addresses
    this.fingerJointAddresses = [];
    for (const jointName of this.fingerJointNames) {
      const address = this.getJointAddress(jointName);
      if (address >= 0) {
        this.fingerJointAddresses.push({ name: jointName, address });
      } else {
        console.warn(`[GripperOpenChecker] Joint "${jointName}" not found`);
      }
    }
    
    if (this.fingerJointAddresses.length === 0) {
      throw new Error('[GripperOpenChecker] No finger joints found!');
    }
    
    console.log(`[GripperOpenChecker] Initialized with ${this.fingerJointAddresses.length} finger joints, threshold=${this.threshold}`);
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
   * Check if gripper is open (joint angle > threshold)
   * @returns {boolean} True if gripper is open, false otherwise
   */
  check() {
    if (this.fingerJointAddresses.length === 0) {
      return false;
    }
    
    // Check all finger joints - gripper is open if ANY joint angle > threshold
    for (const { name, address } of this.fingerJointAddresses) {
      if (address >= 0 && address < this.data.qpos.length) {
        const angle = this.data.qpos[address];
        if (angle > this.threshold) {
          return true;
        }
      }
    }
    
    return false;
  }
  
  /**
   * Get current gripper angles
   * @returns {Object} Object with joint names and their current angles
   */
  getGripperAngles() {
    const angles = {};
    for (const { name, address } of this.fingerJointAddresses) {
      if (address >= 0 && address < this.data.qpos.length) {
        angles[name] = this.data.qpos[address];
      }
    }
    return angles;
  }
  
  /**
   * Get detailed status
   * @returns {Object} Status object with check result and details
   */
  getStatus() {
    const angles = this.getGripperAngles();
    const isOpen = this.check();
    const maxAngle = Math.max(...Object.values(angles));
    
    return {
      isOpen,
      threshold: this.threshold,
      angles,
      maxAngle,
      message: isOpen 
        ? `Gripper is OPEN (max angle: ${maxAngle.toFixed(4)} > ${this.threshold})`
        : `Gripper is CLOSED (max angle: ${maxAngle.toFixed(4)} <= ${this.threshold})`
    };
  }
}

