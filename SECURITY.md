# Security Policy

## Supported Scope

AxisWebInfra is a frontend-only MuJoCo teleoperation demo. It records and exports
trajectories in the browser and can automatically mark a local task as complete
when the configured checker reports success.

That local completion state is not a security boundary. Do not use a browser
completion flag, client-supplied `is_completed`, client-supplied
`goal_achieved`, or downloadable trajectory JSON as the source of truth for
accounts, leaderboards, payouts, rewards, or reputation.

## Production Completion Verification

Production systems must verify task completion on a trusted backend:

- issue a server-side attempt id and one-time submit nonce before each run
- bind the nonce to the authenticated user, task id, task version, checker
  version, scene version, and randomized initial state
- reject replays, expired nonces, duplicate trajectory hashes, oversized
  payloads, and malformed samples
- replay or evaluate the submitted trajectory in a controlled server environment
  using the canonical checker configuration
- ignore client-supplied completion booleans and compute the completion result
  server-side
- compute points, rewards, ranks, and badges only after backend verification

See [Task Completion Verification](docs/task-completion-verification.md) for the
recommended contract.

## Reporting Vulnerabilities

If you find a vulnerability in a production deployment, avoid public exploit
details and report it privately to the project maintainers. Include the affected
endpoint, impact, a minimal non-destructive reproduction, and the expected fix.
