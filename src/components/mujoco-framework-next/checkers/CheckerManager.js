/**
 * CheckerManager
 * Manages task checkers dynamically based on configuration.
 * Main.js should only interact with CheckerManager, not individual checkers.
 */

import { getCheckerClass, CHECKER_REGISTRY } from './index.js';
import { CompositeChecker } from './CompositeChecker.js';

export class CheckerManager {
  constructor(mujoco, model, data, checkerConfig, context = {}) {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;
    this.checkerConfig = checkerConfig;
    this.context = context;
    this.checkers = [];
    this.mainChecker = null;
    
    if (checkerConfig) {
      this.setup();
    }
  }
  
  setup() {
    if (!this.checkerConfig) {
      console.log('[CheckerManager] No checker config provided from task');
      return;
    }
    
    console.log('[CheckerManager] Setting up checkers from task config:', JSON.stringify(this.checkerConfig, null, 2));
    
    try {
      // Format 1: { checker: { type: ... } } - nested checker field
      if (this.checkerConfig.checker) {
        console.log('[CheckerManager] Creating checker from task config.checker');
        this.mainChecker = this.createChecker(this.checkerConfig.checker);
        console.log('[CheckerManager] Main checker created:', this.mainChecker.constructor?.name);
      }
      // Format 2: { type: ... } - direct checker config (from tasks.json)
      else if (this.checkerConfig.type) {
        console.log('[CheckerManager] Creating checker from direct task config (has type field)');
        this.mainChecker = this.createChecker(this.checkerConfig);
        console.log('[CheckerManager] Main checker created:', this.mainChecker.constructor?.name);
      }
      // Format 3: Legacy format - multiple checkers as object keys
      else {
        console.log('[CheckerManager] Using legacy format, creating individual checkers from task config');
        this.setupLegacyCheckers();
      }
    } catch (error) {
      console.error('[CheckerManager] Failed to setup checkers from task config:', error);
      throw error;
    }
  }
  
  setupLegacyCheckers() {
    // Legacy: create individual checkers from config
    for (const [key, config] of Object.entries(this.checkerConfig)) {
      if (config && typeof config === 'object' && config.enabled !== false) {
        try {
          // Try to infer checker type from key name (e.g., "gripperOpenChecker" -> "GripperOpenChecker")
          const typeName = key.charAt(0).toUpperCase() + key.slice(1);
          const checker = this.createCheckerInstance(typeName, config);
          if (checker) {
            this.checkers.push({ name: key, instance: checker });
          }
        } catch (error) {
          console.warn(`[CheckerManager] Failed to create checker ${key}:`, error);
        }
      }
    }
  }
  
  createChecker(config) {
    if (!config || !config.type) {
      throw new Error('[CheckerManager] Checker config from task must have a "type" field. Config: ' + JSON.stringify(config));
    }
    
    console.log(`[CheckerManager] Creating checker of type "${config.type}" from task config`);
    
    // Handle composite checker (special case - doesn't need registry lookup)
    if (config.type === 'CompositeChecker' || config.type === 'composite') {
      if (!config.checkers || !Array.isArray(config.checkers) || config.checkers.length === 0) {
        throw new Error('[CheckerManager] CompositeChecker from task config must have at least one sub-checker in "checkers" array');
      }
      
      console.log(`[CheckerManager] Creating CompositeChecker with ${config.checkers.length} sub-checkers, operator="${config.operator || 'AND'}"`);
      const subCheckers = config.checkers.map((subConfig, index) => {
        console.log(`[CheckerManager] Creating sub-checker ${index + 1}/${config.checkers.length} from task config`);
        return this.createChecker(subConfig);
      });
      
      return new CompositeChecker(this.mujoco, this.model, this.data, {
        operator: config.operator || 'AND',
        checkers: subCheckers
      });
    }
    
    // Handle single checker (lookup from registry)
    return this.createCheckerInstance(config.type, config);
  }
  
  createCheckerInstance(typeName, config) {
    const CheckerClass = getCheckerClass(typeName);
    if (!CheckerClass) {
      throw new Error(`[CheckerManager] Unknown checker type "${typeName}" from task config. Available types: ${Object.keys(CHECKER_REGISTRY).join(', ')}`);
    }
    
    // Remove 'type' and 'enabled' from config before passing to checker
    // All other options come from task config
    const { type, enabled, ...checkerOptions } = config;
    const mergedOptions = { ...checkerOptions, ...this.context };
    
    console.log(`[CheckerManager] Creating ${typeName} with options from task config:`, checkerOptions);
    
    return new CheckerClass(this.mujoco, this.model, this.data, mergedOptions);
  }
  
  /**
   * Check if task success conditions are met
   * Main.js should only call this method
   */
  check() {
    if (this.mainChecker) {
      return this.mainChecker.check();
    }
    
    // Legacy: check all individual checkers (default to AND logic)
    if (this.checkers.length === 0) {
      return false;
    }
    
    return this.checkers.every(({ instance }) => {
      try {
        return instance.check ? instance.check() : false;
      } catch (error) {
        console.warn('[CheckerManager] Error checking checker:', error);
        return false;
      }
    });
  }
  
  /**
   * Get detailed status of all checkers
   */
  getStatus() {
    if (this.mainChecker) {
      if (this.mainChecker.getStatus) {
        return this.mainChecker.getStatus();
      }
      return {
        type: this.mainChecker.constructor?.name || 'Unknown',
        checked: this.mainChecker.check ? this.mainChecker.check() : false
      };
    }
    
    // Legacy: return status of all individual checkers
    return {
      checkers: this.checkers.map(({ name, instance }) => ({
        name,
        type: instance.constructor?.name || 'Unknown',
        checked: instance.check ? instance.check() : false,
        status: instance.getStatus ? instance.getStatus() : null
      }))
    };
  }

  /**
   * Get a lightweight summary of which success conditions are NOT met.
   * Designed for periodic console logging.
   */
  getUnmetConditionsSummary() {
    const status = this.getStatus();
    const missing = new Set();

    const handleLeaf = (type, leafStatus) => {
      if (!type) return;
      if (!leafStatus || typeof leafStatus !== 'object') return;

      // Fallback: if checker reports success=false, include its type
      if (leafStatus.success === false) {
        missing.add(type);
      }
    };

    const walk = (node) => {
      if (!node) return;

      // CompositeChecker.getStatus() shape
      if (node.operator && Array.isArray(node.checkers)) {
        for (const entry of node.checkers) {
          const type = entry?.type || entry?.name || 'Unknown';
          const sub = entry?.status ?? entry;
          if (sub && sub.operator && Array.isArray(sub.checkers)) {
            walk(sub);
          } else {
            handleLeaf(type, sub);
          }
        }
        return;
      }

      // CheckerManager legacy shape
      if (Array.isArray(node.checkers)) {
        for (const entry of node.checkers) {
          const type = entry?.type || entry?.name || 'Unknown';
          const sub = entry?.status ?? entry;
          if (sub && sub.operator && Array.isArray(sub.checkers)) {
            walk(sub);
          } else {
            handleLeaf(type, sub);
          }
        }
        return;
      }

      // Single checker status shape
      if (node.type && node.checked !== undefined) {
        handleLeaf(node.type, node.status ?? node);
        return;
      }
    };

    walk(status);
    return { missing: Array.from(missing) };
  }

  /**
   * Get an extensible progress report for periodic logging.
   * Each leaf checker may implement getProgress() -> { type, goals: [...] }.
   * We flatten CompositeChecker trees into a single list of goals.
   *
   * @returns {{
   *   overall: boolean,
   *   goals: Array<{ key: string, label: string, ok: boolean, current?: number, threshold?: number, unit?: string, type?: string }>
   * }}
   */
  getProgressReport() {
    const goals = [];

    const pushFallback = (checker) => {
      try {
        const type = checker?.constructor?.name || 'Unknown';
        if (checker?.getStatus) {
          const st = checker.getStatus();
          const ok = st?.success ?? st?.checked ?? checker.check?.() ?? false;
          goals.push({ key: type, label: type, ok: !!ok, type });
        } else {
          const ok = checker?.check ? checker.check() : false;
          goals.push({ key: type, label: type, ok: !!ok, type });
        }
      } catch (e) {
        const type = checker?.constructor?.name || 'Unknown';
        goals.push({ key: type, label: type, ok: false, type });
      }
    };

    const walkInstance = (checker) => {
      if (!checker) return;
      // CompositeChecker
      if (checker instanceof CompositeChecker || (checker.checkers && Array.isArray(checker.checkers) && checker.operator)) {
        for (const sub of checker.checkers || []) {
          walkInstance(sub);
        }
        return;
      }
      // Leaf checker
      if (typeof checker.getProgress === 'function') {
        try {
          const pr = checker.getProgress();
          const type = pr?.type || checker.constructor?.name || 'Unknown';
          const leafGoals = Array.isArray(pr?.goals) ? pr.goals : [];
          if (leafGoals.length > 0) {
            for (const g of leafGoals) {
              goals.push({
                key: String(g.key ?? `${type}`),
                label: String(g.label ?? g.key ?? type),
                ok: !!g.ok,
                current: typeof g.current === 'number' ? g.current : undefined,
                threshold: typeof g.threshold === 'number' ? g.threshold : undefined,
                unit: g.unit ? String(g.unit) : undefined,
                type,
              });
            }
            return;
          }
        } catch (e) {
          // fall through to fallback
        }
      }
      pushFallback(checker);
    };

    if (this.mainChecker) {
      walkInstance(this.mainChecker);
    } else if (this.checkers.length > 0) {
      for (const { instance } of this.checkers) {
        walkInstance(instance);
      }
    }

    const overall = this.check();
    return { overall: !!overall, goals };
  }
  
  /**
   * Update checkers when model/data changes
   */
  updateModelData(model, data) {
    this.model = model;
    this.data = data;
    
    // Update all checker instances if they have updateModelData method
    if (this.mainChecker && this.mainChecker.updateModelData) {
      this.mainChecker.updateModelData(model, data);
    }
    
    for (const { instance } of this.checkers) {
      if (instance.updateModelData) {
        instance.updateModelData(model, data);
      }
    }
  }
}
