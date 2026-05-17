# Task Completion Verification

This document defines the security contract for deployments that use MuJoCo task
completion for accounts, leaderboards, rewards, payouts, or reputation.

## Threat Model

All browser state is attacker-controlled. A user can edit JavaScript, replace
callbacks, forge exported trajectory JSON, replay an old payload, change local
storage, alter timing metadata, and submit arbitrary request bodies.

The frontend may provide useful evidence, but it must never be trusted as the
authority for task completion.

## Required Backend Contract

1. Create an attempt before the simulation starts.

   The backend returns an `attempt_id` and one-time `submit_nonce` only to an
   authenticated user. The attempt must be bound to:

   - user id
   - task id and task version
   - scene asset version
   - checker version
   - deterministic randomization seed
   - expiration time

2. Make randomization deterministic and server-owned.

   If the client needs a nonce or seed to render the same scene, treat that
   value as public but signed or server-recorded. On submit, the backend must
   reconstruct the same initial state from server records.

3. Accept trajectories as evidence, not proof.

   The submit endpoint should accept trajectory samples and metadata, but it
   must ignore client-supplied fields such as `is_completed`, `goal_achieved`,
   `points`, `score`, `rank`, `bonus`, and `reward`.

4. Validate payload shape before simulation.

   Reject payloads that violate:

   - maximum compressed and decompressed size
   - maximum sample count and duration
   - monotonic timestamps
   - finite numeric values only
   - expected joint, control, object, and action schema
   - joint limits, velocity limits, and control limits
   - task id and nonce consistency

5. Verify completion in a controlled backend worker.

   The backend should replay or evaluate the submitted trajectory against the
   canonical MJCF, initial state, domain randomization, and checker config. The
   verifier returns `verified`, `failed`, or `needs_review` with a reason.

6. Store immutable verification evidence.

   Store the trajectory hash, verifier version, checker version, task version,
   result, failure reason, and timing. Use idempotency so repeated submissions
   for the same attempt cannot produce multiple rewards.

7. Award only after verification.

   Points, rewards, badges, leaderboard entries, referrals, and progress should
   be computed exclusively from backend-verified attempts.

## Submit Endpoint Behavior

Recommended behavior for `POST /api/tasks/{task_id}/complete`:

- require an authenticated session
- require an active attempt owned by the user
- require a valid one-time `submit_nonce`
- reject already-consumed or expired nonces
- reject task id mismatches
- enqueue verification for large trajectories
- return a pending or verified attempt id
- never mark completion solely because the request body says it is complete

## Example Verification Flow

```text
client:  GET /api/tasks/{task_id}
server:  returns task data plus attempt_id and submit_nonce
client:  runs MuJoCo locally and exports trajectory
client:  POST /api/tasks/{task_id}/complete with attempt_id, submit_nonce, trajectory
server:  validates auth, nonce, payload limits, and schema
server:  replays/evaluates trajectory with canonical checker
server:  stores verified/failed result and consumes nonce
server:  awards progress only for verified attempts
```

## Frontend Responsibilities

The frontend should still help users and reduce accidental bad submissions:

- include attempt id and nonce in trajectory metadata
- send task id, task version, checker version, and scene version
- preserve monotonic timestamps and sampling interval metadata
- avoid showing rewards until backend verification succeeds
- display failed or pending verification states clearly

These checks improve user experience but are not security controls by
themselves.
