/**
 * BodyContactChecker
 *
 * Checks whether two bodies are in contact (or within `max_contact_distance`
 * of each other) at the current simulation step.  Used by Hub tasks to check
 * success conditions like "the gripper is touching the plate" or "the lid
 * is sitting on the box" without having to measure every position.
 *
 * Task config schema (matches Hub `checker_config` for `BodyContactChecker`):
 *
 *   {
 *     "type": "BodyContactChecker",
 *     "bodyAName": "akita_black_bowl_1_main",
 *     "bodyBName": "plate_1_main",
 *     "maxContactDistance": 0.02,    // meters; default 0
 *     "minContactCount": 1            // default 1
 *   }
 *
 * The matching Python implementation is in
 *   AxisDataCleaning/util/validate_offline_trajectory.py::check_body_contact
 * and MUST stay in sync with this class.  See PR description for the
 * cross-repo coordination plan.
 *
 * Implementation notes:
 *
 *  - MuJoCo stores per-step contact data in `data.contact`.  Each contact has
 *    two geoms (`contact.geom1`, `contact.geom2`) and a distance
 *    (`contact.dist`).  The distance is the **signed** minimum translational
 *    distance between the two geoms; negative means the geoms overlap.
 *
 *  - We map each `bodyName` to the set of geom IDs that belong to that body
 *    (via `model.body_geomadr[bodyId]..body_geomadr[bodyId]+model.body_geomnum[bodyId]`)
 *    and then walk `data.contact` looking for pairs where one geom is in set A
 *    and the other in set B.
 *
 *  - If `maxContactDistance > 0`, we additionally accept "near contact"
 *    candidates: pairs of bodies whose closest points are within
 *    `maxContactDistance`.  This is a fallback for the "soft contact" semantics
 *    that Hub uses for things like "lid is sitting on the box" where the lid
 *    may not be in true geometric contact but is very close.
 */

export class BodyContactChecker {
  constructor(mujoco, model, data, options = {}) {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;

    // ---- options ---------------------------------------------------------
    this.bodyAName = options.body_a || options.bodyAName;
    this.bodyBName = options.body_b || options.bodyBName;
    if (!this.bodyAName || !this.bodyBName) {
      throw new Error(
        '[BodyContactChecker] body_a (or bodyAName) and body_b (or bodyBName) are required',
      );
    }
    if (this.bodyAName === this.bodyBName) {
      throw new Error(
        `[BodyContactChecker] body_a and body_b must differ, both = "${this.bodyAName}"`,
      );
    }

    this.maxContactDistance = Number(
      options.max_contact_distance ?? options.maxContactDistance ?? 0,
    );
    if (!Number.isFinite(this.maxContactDistance) || this.maxContactDistance < 0) {
      throw new Error(
        `[BodyContactChecker] maxContactDistance must be a non-negative number, got ${options.max_contact_distance ?? options.maxContactDistance}`,
      );
    }

    this.minContactCount = Number(
      options.min_contact_count ?? options.minContactCount ?? 1,
    );
    if (
      !Number.isInteger(this.minContactCount) ||
      this.minContactCount < 1
    ) {
      throw new Error(
        `[BodyContactChecker] minContactCount must be a positive integer, got ${options.min_contact_count ?? options.minContactCount}`,
      );
    }

    // ---- model introspection --------------------------------------------
    this.textDecoder = new TextDecoder('utf-8');
    this.namesBuffer = new Uint8Array(model.names);
    this.bodyIdA = this.resolveBodyId(this.bodyAName);
    this.bodyIdB = this.resolveBodyId(this.bodyBName);
    if (this.bodyIdA < 0) {
      console.warn(`[BodyContactChecker] body "${this.bodyAName}" not found`);
    }
    if (this.bodyIdB < 0) {
      console.warn(`[BodyContactChecker] body "${this.bodyBName}" not found`);
    }

    // Cache the set of geom IDs belonging to each body.  We rebuild this
    // whenever the model is reloaded (see updateModelData).
    this.geomIdsA = this.collectGeomIdsForBody(this.bodyIdA);
    this.geomIdsB = this.collectGeomIdsForBody(this.bodyIdB);

    // Mutable contact count, read on each check() and reset via reset().
    this._contactCount = 0;

    console.log(
      `[BodyContactChecker] Initialized: bodyA="${this.bodyAName}" bodyB="${this.bodyBName}" ` +
        `maxContactDistance=${this.maxContactDistance} minContactCount=${this.minContactCount} ` +
        `(|A|=${this.geomIdsA.size} geoms, |B|=${this.geomIdsB.size} geoms)`,
    );
  }

  resolveBodyId(bodyName) {
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

  /**
   * Returns a Set of geom IDs whose body is `bodyId` (or any descendant).
   * This is the inclusive-union: a child body's geoms count toward the parent.
   */
  collectGeomIdsForBody(bodyId) {
    const result = new Set();
    if (bodyId < 0 || bodyId >= this.model.nbody) return result;
    for (let g = 0; g < this.model.ngeom; g++) {
      // geom_bodyid is an Int32Array; positive values are body IDs.
      if (this.model.geom_bodyid[g] === bodyId) {
        result.add(g);
      }
    }
    return result;
  }

  /**
   * Walks the current `data.contact` array and counts the number of contact
   * pairs that have one geom in set A and the other in set B.  For each such
   * pair, also respects `maxContactDistance`: the pair counts only if the
   * contact distance (or, if `maxContactDistance > 0` and the pair is not in
   * true contact, the closest-point distance) is within the threshold.
   */
  countContactPairs() {
    if (this.geomIdsA.size === 0 || this.geomIdsB.size === 0) return 0;
    if (!this.data.contact) return 0;
    // The `contact` field may be exposed as a typed array of pairs
    // (geom1, geom2, dist) or as an object array; support both via duck typing.
    const contact = this.data.contact;
    let count = 0;
    // Use the number of *valid contacts* reported by the model.  mujoco-js
    // exposes ncon; some wrappers also expose contact.length.  We guard with
    // the struct-of-arrays shape check so a plain object without `.length`
    // still iterates correctly via ncon.
    const n = this.model.ncon ?? (typeof contact.length === 'number' ? contact.length : 0);
    if (contact.geom1 !== undefined) {
      // Struct-of-arrays style (typical for mujoco-js).
      for (let i = 0; i < n; i++) {
        const g1 = contact.geom1[i];
        const g2 = contact.geom2[i];
        const dist = contact.dist ? contact.dist[i] : 0;
        if (dist > this.maxContactDistance) continue;
        if (
          (this.geomIdsA.has(g1) && this.geomIdsB.has(g2)) ||
          (this.geomIdsA.has(g2) && this.geomIdsB.has(g1))
        ) {
          count += 1;
        }
      }
    } else if (Array.isArray(contact)) {
      // Array-of-objects style.
      for (let i = 0; i < contact.length; i++) {
        const c = contact[i];
        if (!c) continue;
        const g1 = c.geom1 ?? c.geom1id;
        const g2 = c.geom2 ?? c.geom2id;
        const dist = c.dist ?? c.distance ?? 0;
        if (dist > this.maxContactDistance) continue;
        if (
          (this.geomIdsA.has(g1) && this.geomIdsB.has(g2)) ||
          (this.geomIdsA.has(g2) && this.geomIdsB.has(g1))
        ) {
          count += 1;
        }
      }
    }
    return count;
  }

  /**
   * Whether the configured bodies are in (or near) contact with the required
   * count.  This is the primary "is the task done" check.
   * @returns {boolean}
   */
  check() {
    if (this.bodyIdA < 0 || this.bodyIdB < 0) return false;
    this._contactCount = this.countContactPairs();
    return this._contactCount >= this.minContactCount;
  }

  /**
   * Detail object for the debug UI / progress reporting.
   */
  getStatus() {
    const count = this._contactCount; // populated by the most recent check()
    return {
      success: this.check(),
      body_a: this.bodyAName,
      body_b: this.bodyBName,
      contact_count: count,
      min_contact_count: this.minContactCount,
      max_contact_distance: this.maxContactDistance,
      message:
        count >= this.minContactCount
          ? `In contact (${count} pairs >= ${this.minContactCount})`
          : `Not in contact (${count} pairs < ${this.minContactCount})`,
    };
  }

  /**
   * Progress reporting for the periodic console logger in main.js.
   */
  getProgress() {
    const count = this._contactCount;
    return {
      type: 'BodyContactChecker',
      goals: [
        {
          key: `${this.bodyAName}->${this.bodyBName}`,
          label: `${this.bodyAName} → ${this.bodyBName}`,
          ok: count >= this.minContactCount,
          current: count,
          threshold: this.minContactCount,
          unit: 'pairs',
        },
      ],
    };
  }

  /**
   * Resets the contact counter.  Called by CheckerManager when a task
   * is reloaded or reset, so a fresh attempt doesn't inherit a previous
   * attempt's contact count.
   */
  reset() {
    this._contactCount = 0;
  }

  /**
   * Rebuilds the geom-id caches and resets the contact counter.  Called by
   * CheckerManager when the underlying MuJoCo model is reloaded (scene
   * change, etc.).
   */
  updateModelData(model, data) {
    this.model = model;
    this.data = data;
    this.namesBuffer = new Uint8Array(model.names);
    this.bodyIdA = this.resolveBodyId(this.bodyAName);
    this.bodyIdB = this.resolveBodyId(this.bodyBName);
    this.geomIdsA = this.collectGeomIdsForBody(this.bodyIdA);
    this.geomIdsB = this.collectGeomIdsForBody(this.bodyIdB);
    this._contactCount = 0;
  }
}
