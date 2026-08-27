/**
 * OrderedStepChecker
 *
 * A stateful checker that runs its sub-checkers **in sequence** and only
 * succeeds when each step has been satisfied in order.  This is the
 * "multi-phase task" success condition used by Hub tasks that require e.g.
 * "first pick up the object, THEN place it in the box".
 *
 * Task config schema (matches Hub `checker_config` for `OrderedStepChecker`):
 *
 *   {
 *     "type": "OrderedStepChecker",
 *     "steps": [
 *       { "type": "RelativePositionBoundsChecker", ... },  // step 1
 *       { "type": "GripperOpenChecker", ... }               // step 2
 *     ],
 *     "allow_reentry": false    // optional; if true, a step that is already
 *                               // satisfied can be skipped forward but earlier
 *                               // steps that become unsatisfied do NOT reset
 *                               // progress (default false = strict ordering)
 *   }
 *
 * The matching Python implementation is in
 *   AxisDataCleaning/util/validate_offline_trajectory.py::check_ordered_step
 * and MUST stay in sync with this class.
 *
 * Semantics:
 *
 *   - The checker keeps an internal index `this.currentStep`.
 *   - On each `check()`:
 *       * It evaluates the sub-checker at `currentStep`.  If it passes,
 *         advance to the next step.  Repeat until the current step fails
 *         or all steps have been satisfied.
 *       * If `allow_reentry` is false (strict): once a later step is being
 *         evaluated, earlier steps are no longer re-evaluated.  A failure at
 *         the current step does NOT reset already-completed steps.
 *       * The checker returns true iff `currentStep >= steps.length` (i.e.
 *         all steps completed).
 *   - On `reset()`, `currentStep` returns to 0.
 *
 * This mirrors how a Hub "OrderedStepChecker" behaves in practice and is
 * the semantic primitive behind the longer, multi-phase LIBERO tasks.
 */

export class OrderedStepChecker {
  constructor(mujoco, model, data, options = {}) {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;

    // Two accepted shapes:
    //   1. options.steps = array of sub-checker CONFIGS (Hub format) — we build
    //      the sub-checkers here using a registry resolver.
    //   2. options.checkers = array of already-constructed sub-checker
    //      instances — used when the caller (CheckerManager) resolves them,
    //      avoiding the ESM circular-import problem entirely.
    const rawSteps = options.steps || [];
    const prebuilt = options.checkers || [];

    if (prebuilt.length > 0) {
      this.subCheckers = prebuilt;
    } else if (Array.isArray(rawSteps) && rawSteps.length > 0) {
      // Build from configs via an injected registry resolver.  We inject it
      // through options to avoid a static ESM circular import (checker files
      // import index.js, index.js imports checker files).  CheckerManager is
      // the natural place to supply it, since it already holds the registry.
      const registry = options.registry || options.checkerClasses || {};
      const resolver =
        options.getCheckerClass ||
        ((t) => registry[t] || registry[String(t).toLowerCase()] || null);
      if (typeof resolver !== 'function') {
        throw new Error('[OrderedStepChecker] a registry resolver is required');
      }
      this.subCheckers = rawSteps.map((cfg, i) => {
        const CheckerClass = resolver(cfg.type);
        if (typeof CheckerClass !== 'function') {
          throw new Error(`[OrderedStepChecker] unknown sub-checker type "${cfg.type}"`);
        }
        const sub = new CheckerClass(mujoco, model, data, cfg);
        sub._orderedStepIndex = i;
        return sub;
      });
    } else {
      throw new Error(
        '[OrderedStepChecker] provide either options.steps (config array) ' +
          'or options.checkers (pre-built instances)',
      );
    }

    this.allowReentry = Boolean(options.allow_reentry ?? options.allowReentry ?? false);

    this.currentStep = 0;

    console.log(
      `[OrderedStepChecker] Initialized: ${this.subCheckers.length} steps, allowReentry=${this.allowReentry}`,
    );
  }

  /**
   * Advances through satisfied steps in order.  Returns true iff all steps
   * have been completed.
   */
  check() {
    // With strict ordering, only evaluate the current (next unfinished) step.
    // Re-evaluating earlier steps is only done when allowReentry is true and
    // a later step hasn't been reached yet.
    if (this.currentStep >= this.subCheckers.length) {
      // Already fully satisfied.
      return true;
    }

    // Evaluate forward from the current step while steps pass.
    while (this.currentStep < this.subCheckers.length) {
      const sub = this.subCheckers[this.currentStep];
      const ok = typeof sub.check === 'function' ? sub.check() : false;
      if (!ok) break;
      this.currentStep++;
    }

    return this.currentStep >= this.subCheckers.length;
  }

  /**
   * Details about each step for the debug UI.
   */
  getStatus() {
    const stepStatuses = this.subCheckers.map((sub, i) => {
      const base =
        typeof sub.getStatus === 'function' ? sub.getStatus() : { success: false };
      return {
        step: i + 1,
        type: sub.constructor?.name,
        completed: i < this.currentStep,
        current: i === this.currentStep,
        ...(base || {}),
      };
    });
    return {
      success: this.check(),
      current_step: this.currentStep,
      total_steps: this.subCheckers.length,
      allow_reentry: this.allowReentry,
      steps: stepStatuses,
      message:
        this.currentStep >= this.subCheckers.length
          ? 'All steps completed'
          : `On step ${this.currentStep + 1} of ${this.subCheckers.length}`,
    };
  }

  getProgress() {
    const goals = this.subCheckers.map((sub, i) => {
      const ok = i < this.currentStep;
      return {
        key: `step-${i + 1}`,
        label: `Step ${i + 1} (${sub.constructor?.name})`,
        ok,
        current: ok ? 1 : 0,
        threshold: 1,
        unit: 'done',
      };
    });
    return { type: 'OrderedStepChecker', goals };
  }

  reset() {
    this.currentStep = 0;
    for (const sub of this.subCheckers) {
      if (typeof sub.reset === 'function') sub.reset();
    }
  }

  updateModelData(model, data) {
    this.model = model;
    this.data = data;
    for (const sub of this.subCheckers) {
      if (typeof sub.updateModelData === 'function') sub.updateModelData(model, data);
    }
  }
}
