# Committed/backup execution and recovery FSM

Implemented for enable_tracking_commands=true + tracking_guarded.
Ordinary ReplanOnce FAILED retains the last committed snapshot and retries on
the configured replan timer. The snapshot is the original SUPER CmdTraj position
polynomial, including its appended backup. No new trajectory is invented after
failure and the old time origin is preserved.

The command sampler runs independently at 50Hz, uses the original polynomial
until completion, then holds its terminal position with zero velocity. The lease
is refreshed by healthy supervision ticks (fresh odometry and bounded tracking
error), rather than successful optimization. Thus repeated optimizer failures do
not manufacture a command timeout; a stalled supervision callback still expires
after 2.5 wall seconds.

After completion, measured speed <=0.2 m/s for 0.5 s permits PlanFromRest. If the
vehicle is within 0.2 m of the active goal, it holds at the goal instead. Otherwise
it retries the same goal, keeping terminal commands alive. Failed rest plans retry
at 1 Hz; successful ones replace the snapshot. An explicitly invalid goal stops
retries while preserving the terminal hold. A moving replacement goal waits for
the old committed stop before planning from rest.

Safety faults (odometry loss, >0.5m tracking error, clock reset, emergency return,
invalid/nonstopping committed trajectory) clear execution and require a new goal.
Queued old goals are discarded on safety halt. The flight manager's existing
command/LIO/PX4 watchdogs and manual/landing/LIO latch priority are unchanged.
Preview mode retains its stop-on-failed-replan behavior.

## Validation

- CTest committed_execution: uses the real SUPER ExpTraj/BackupTraj/CmdTraj
  concatenation; validates polynomial continuity through backup and terminal hold
  across ten simulated seconds without successful replacement, supervision
  expiry, clock reset, explicit clear, and rejection of a nonstopping endpoint.
- CTest replanner_recovery: invokes production node tick()/stream_command() with
  deterministic planner/clock/odometry inputs. Collects 651 successive Odometry
  commands, each separated by 0.02 s, through repeated failed replans, backup end,
  two failed PlanFromRest attempts, successful restart, and final goal hold.
  Also checks motion/settling gate, invalid-goal hold, moving goal replacement,
  success/finish/new-trajectory returns, preview behavior and safety faults.
- CTest backup_constraints: preserves real optimizer regression cases for the
  earlier backup configuration/constraint fixes.
- Python tracking and LIO ROS tests: 19 passed.
- Isolated-domain launch smoke: tracking_guarded + commands enabled initialized;
  stopped deliberately by timeout SIGINT. No PX4 commands or arm/takeoff.

C++ ROS tests and launch smoke use ROS_DOMAIN_ID=87, independent of the flight
stack. The test fixture publishes only /test_recovery/* topics; it does not run
ROG-Map, inject failures into the user's running planner or touch /fmu/in/*.

This validates implementation behavior, not actual PX4 vehicle dynamics or the
user's obstacle map. A closed-loop SITL run must still verify deceleration,
continued command rate, stationary restart, and manual/LIO interruption.

## Upstream correspondence

Based on the previously inspected official revision
2ad3419c127a617c6d7df6925e81a14175a9c096: ordinary FAILED leaves FOLLOW_TRAJ active;
command timer continues evaluating committed trajectory; after completion away
from goal, FSM returns to GENERATE_TRAJ. ZAV adds an explicit measured-speed settle
gate and fail-closed sensor/error/clock handling around the existing safety gateway.
