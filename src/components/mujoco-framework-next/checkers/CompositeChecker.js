/**
 * CompositeChecker
 * Combines multiple checkers with logical operators (AND/OR)
 */
export class CompositeChecker {
  constructor(mujoco, model, data, options = {}) {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;
    
    // Configuration
    this.operator = options.operator ?? 'AND'; // 'AND' or 'OR'
    this.checkers = options.checkers ?? []; // Array of checker instances
    
    if (this.checkers.length === 0) {
      throw new Error('[CompositeChecker] No checkers provided');
    }
    
    console.log(`[CompositeChecker] Initialized with ${this.checkers.length} checkers, operator=${this.operator}`);
  }
  
  check() {
    if (this.checkers.length === 0) {
      return false;
    }
    
    const results = this.checkers.map(checker => {
      try {
        return checker.check ? checker.check() : false;
      } catch (error) {
        console.warn('[CompositeChecker] Error checking sub-checker:', error);
        return false;
      }
    });
    
    if (this.operator === 'AND') {
      return results.every(r => r === true);
    } else if (this.operator === 'OR') {
      return results.some(r => r === true);
    } else {
      console.warn(`[CompositeChecker] Unknown operator: ${this.operator}, defaulting to AND`);
      return results.every(r => r === true);
    }
  }
  
  getStatus() {
    const statuses = this.checkers.map((checker, index) => {
      try {
        return {
          index,
          type: checker.constructor?.name || 'Unknown',
          status: checker.getStatus ? checker.getStatus() : { checked: checker.check ? checker.check() : false }
        };
      } catch (error) {
        return {
          index,
          type: checker.constructor?.name || 'Unknown',
          error: error.message
        };
      }
    });
    
    return {
      operator: this.operator,
      overall: this.check(),
      checkers: statuses
    };
  }
}







