/**
 * PositionDeltaChecker
 *
 * Checks whether a body's position has changed by at least a given
 * threshold along one or more axes relative to a captured reference
 * position.  Used by Hub tasks to detect coarse-grained motion such as
 * "the lid was rotated past 30 degrees" or "the object was moved at
 * least 2 cm to the left of where it started".
 *
 * Task config schema (matches Hub `checker_config` for `PositionDeltaChecker`):
 *
 *   {
 *     "type": "PositionDeltaChecker",
 *     "sampleBodyName": "akita_black_bowl_1_main",
 *     "axes": ["x", "y"],                // which axes to consider
 *     "minDeltaX": 0.018,                // body must have moved +18mm on X
 *     "maxDeltaX": -0.018,               // OR must have moved -18mm on X
 *     "minDeltaY": 0.018,                // same for Y
 *     "maxDeltaY": -0.018,
 *     "captureRuntimeInitial": true,     // baseline = first-seen body position
 *                                        // false (default) = use task's initial_state
 *     "initialPosition": [x, y, z]       // optional explicit baseline; if absent
 *                                        // and !captureRuntimeInitial, the position
 *                                        // at construction time is used
 *   }
 *
 * The matching Python implementation is in
 *   AxisDataCleaning/util/validate_offline_trajectory.py::check_position_delta
 * and MUST stay in sync with this class.
 *
 * Semantics:
 *
 *   - If `captureRuntimeInitial` is true, the first observed body position
 *     (after the checker is created) is captured as the baseline.  This is
 *     useful when the task's "initial position" in the MJCF does not match
 *     the position the body actually settles into after physics warmup.
 *   - If `captureRuntimeInitial` is false, the baseline is `initialPosition`
 *     if provided, otherwise the position at construction time.
 *   - The check passes if, for ANY of the configured axes, the current
 *     position is >= minDelta (positive direction) OR <= maxDelta (negative
 *     direction).  An axis with neither min nor max bound is ignored.
 *   - The check does NOT require ALL axes to move; OR semantics by default.
 *     If you need AND semantics, wrap in a CompositeChecker.
 */

export class PositionDeltaChecker {
  constructor(mujoco, model, data, options = {}) {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;

    this.bodyName = options.sample_body || options.sampleBodyName;
    if (!this.bodyName) {
      throw new Error(
        '[PositionDeltaChecker] sample_body (or sampleBodyName) is required',
      );
    }

    this.axes = this.parseAxes(options.axes || ['x', 'y']);
    this.captureRuntimeInitial = Boolean(
      options.capture_runtime_initial ?? options.captureRuntimeInitial ?? false,
    );

    // Per-axis bounds.  Either or both may be set per axis; missing means
    // "no constraint on that direction for that axis".
    this.minDeltas = {
      x: this._num(options.min_delta_x, options.minDeltaX),
      y: this._num(options.min_delta_y, options.minDeltaY),
      z: this._num(options.min_delta_z, options.minDeltaZ),
    };
    this.maxDeltas = {
      x: this._num(options.max_delta_x, options.maxDeltaX),
      y: this._num(options.max_delta_y, options.maxDeltaY),
      z: this._num(options.max_delta_z, options.maxDeltaZ),
    };

    this.initialPosition = null;
    this._runtimeBaseline = null;
    if (Array.isArray(options.initialPosition) && options.initialPosition.length === 3) {
      this.initialPosition = [
        Number(options.initialPosition[0]),
        Number(options.initialPosition[1]),
        Number(options.initialPosition[2]),
      ];
    }

    this.textDecoder = new TextDecoder('utf-8');
    this.namesBuffer = new Uint8Array(model.names);
    this.bodyAddress = this.resolveBodyAddress(this.bodyName);
    if (this.bodyAddress < 0) {
      console.warn(`[PositionDeltaChecker] body "${this.bodyName}" not found`);
    }

    console.log(
      `[PositionDeltaChecker] Initialized: body="${this.bodyName}" ` +
        `axes=[${this.axes.join(',')}] captureRuntimeInitial=${this.captureRuntimeInitial} ` +
        `bounds=${JSON.stringify(this._boundsSummary())}`,
    );
  }

  _num(...candidates) {
    for (const v of candidates) {
      if (v === undefined || v === null) continue;
      const n = Number(v);
      if (Number.isFinite(n)) return n;
    }
    return null;
  }

  _boundsSummary() {
    const out = {};
    for (const ax of this.axes) {
      const lo = this.maxDeltas[ax];
      const hi = this.minDeltas[ax];
      if (lo !== null || hi !== null) {
        out[ax] = {};
        if (hi !== null) out[ax].min = hi;
        if (lo !== null) out[ax].max = lo;
      }
    }
    return out;
  }

  parseAxes(axes) {
    if (!Array.isArray(axes)) {
      throw new Error(
        `[PositionDeltaChecker] axes must be an array, got ${typeof axes}`,
      );
    }
    const out = [];
    for (const ax of axes) {
      const a = String(ax).toLowerCase();
      if (!['x', 'y', 'z'].includes(a)) {
        throw new Error(`[PositionDeltaChecker] unknown axis "${ax}"`);
      }
      if (!out.includes(a)) out.push(a);
    }
    if (out.length === 0) {
      throw new Error('[PositionDeltaChecker] axes must contain at least one of x, y, z');
    }
    return out;
  }

  resolveBodyAddress(bodyName) {
    for (let b = 0; b < this.model.nbody; b++) {
      const nameStart = this.model.name_bodyadr[b];
      let nameEnd = nameStart;
      while (
        nameEnd < this.namesBuffer.length &&
        this.namesBuffer[nameEnd] !== 0
      ) {
        nameEnd++;
      }
      const name = this.textDecoder.decode(
        this.namesBuffer.subarray(nameStart, nameEnd),
      );
      if (name === bodyName) return b;
    }
    return -1;
  }

  getCurrentPosition() {
    if (this.bodyAddress < 0 || this.bodyAddress >= this.model.nbody) return null;
    const base = 3 * this.bodyAddress;
    return [
      this.data.xpos[base + 0],
      this.data.xpos[base + 1],
      this.data.xpos[base + 2],
    ];
  }

  /**
   * Returns the current baseline: runtime-captured if available, else the
   * initialPosition given at construction, else the current position
   * (which means delta = 0 and the check will pass if any bound is 0).
   */
  baseline() {
    if (this._runtimeBaseline) return this._runtimeBaseline;
    if (this.initialPosition) return this.initialPosition;
    return this.getCurrentPosition() || [0, 0, 0];
  }

  /**
   * Whether the body has moved past one of the configured bounds.
   */
  check() {
    const cur = this.getCurrentPosition();
    if (!cur) return false;
    let base = this._runtimeBaseline;
    if (!base && this.captureRuntimeInitial) {
      base = [cur[0], cur[1], cur[2]];
      this._runtimeBaseline = base;
    } else if (!base) {
      base = this.baseline();
    }
    for (let i = 0; i < this.axes.length; i++) {
      const ax = this.axes[i];
      const delta = cur[i] - base[i];
      const hi = this.minDeltas[ax];
      const lo = this.maxDeltas[ax];
      if (hi !== null && delta >= hi) return true;
      if (lo !== null && delta <= lo) return true;
    }
    return false;
  }

  /**
   * Current per-axis delta, useful for the debug UI.
   */
  getStatus() {
    const cur = this.getCurrentPosition();
    const base = this.baseline();
    const deltas = { x: 0, y: 0, z: 0 };
    for (let i = 0; i < 3; i++) {
      if (cur && base) deltas[['x','y','z'][i]] = cur[i] - base[i];
    }
    return {
      success: this.check(),
      body: this.bodyName,
      deltas,
      baseline: base,
      current: cur,
      bounds: this._boundsSummary(),
      message: this.check()
        ? `Movement detected for ${this.bodyName}`
        : `${this.bodyName} has not moved past any configured bound`,
    };
  }

  getProgress() {
    const cur = this.getCurrentPosition();
    const base = this.baseline();
    const goals = [];
    for (const ax of this.axes) {
      const idx = { x: 0, y: 1, z: 2 }[ax];
      const delta = cur && base ? cur[idx] - base[idx] : 0;
      const hi = this.minDeltas[ax];
      const lo = this.maxDeltas[ax];
      const passed = (hi !== null && delta >= hi) || (lo !== null && delta <= lo);
      goals.push({
        key: `${this.bodyName}.${ax}`,
        label: `${this.bodyName} Δ${ax}`,
        ok: passed,
        current: delta,
        threshold: hi !== null ? hi : lo ?? 0,
        unit: 'm',
      });
    }
    return { type: 'PositionDeltaChecker', goals };
  }

  reset() {
    this._runtimeBaseline = null;
  }

  updateModelData(model, data) {
    this.model = model;
    this.data = data;
    this.namesBuffer = new Uint8Array(model.names);
    this.bodyAddress = this.resolveBodyAddress(this.bodyName);
    this._runtimeBaseline = null;
  }
}
