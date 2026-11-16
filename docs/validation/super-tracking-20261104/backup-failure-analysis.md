# Backup optimization failure investigation

User log: ReplanOnce failed at 1793801031.937 and 1793801044.436;
flight_manager stopped with stale feedback at 1793801032.225 and 1793801044.710.
The 0.27–0.29 s delay matches the 0.25 s tracking-command watchdog after
replanner intentionally clears its cached command trajectory. Immediately before
failure, tracking errors were 0.07 m and 0.08 m. This evidence points to planning
failure preceding the command timeout, rather than a proven DDS transport fault.

Found in both planner/replanner YAMLs:
- backup_traj defines penna_max_acc_thr and penna_min_acc_thr.
- Config loads only penna_thr, default -1. Both YAML keys were ignored.
- Backup constraints evaluated flatness only if BOTH angular and thrust weights
  were positive. Thus the active angular constraint was skipped too.
- The maximum-violation log then read uninitialized violaOmg/violaThrust;
  the final acceptance check used the angular log. This is undefined behavior.

Fixes:
- Replace ignored keys with penna_thr=100000, matching the config loader.
- Initialize violation values per integration sample; evaluate flatness if either
  constraint is enabled, and apply each penalty independently (backup and exp).
- Record the actual maximum corridor violation (previously remained zero).
- Report constraint residuals/thresholds and enabled weights on backup rejection.
- Distinguish constraint rejection from an unknown LBFGS return code.
- Distinguish command timeout, LIO timeout, and PX4 position timeout in the manager.
- Keep the existing fail-closed hover behavior; do not disable backup or relax limits.

The old logs do not record which numeric constraint tripped; they cannot prove
that the undefined value was the sole cause of these two failures. Real corridor
or dynamics infeasibility can still fail after this correction and will now have
specific diagnostics. No live closed-loop flight was performed for this fix.

Regression tests exercise actual constraint integration with disabled penalties,
angular-only and thrust-only penalties, corridor violation, hover bounds, and
both installed-source YAML parameter sets. Python tests cover per-stream timeout
messages and existing tracking/LIO safety behavior.

Validation result: colcon build zav_super + zav_control passed; CTest
backup_constraints 1/1 passed; Python tracking + LIO ROS tests 19 passed.

## Follow-up: actual acceleration violation after the first correction

New user log: ReplanOnce failure at 1793801419.787553, command timeout at
1793801420.080063. Backup diagnostic: acc_sq=0.290825/0.05;
position=0.00609756/0.2, velocity_sq=0.00176576/0.05, angular_sq=0/0.075,
thrust residual=0/0.05. Acceleration is the rejecting constraint.
For max_acc=1.0, this is approximately sqrt(1+0.290825)=1.136 m/s^2;
the current acceptance threshold is sqrt(1+0.05)=1.025 m/s^2.

The first fix repaired ignored thrust configuration and undefined values but did
not solve this separate, genuine acceleration violation. The optimizer objective
contains penna_ts*(latest_allowed_start - chosen_start), rewarding a late braking
start. At penna_ts=100000 this reward can dominate the acceleration penalty and
produce a candidate subsequently rejected by the unchanged constraint check.

Reproduced without simulator/aircraft commands by invoking the actual backup
optimizer on constant-velocity reference trajectories: velocities 0.5, 0.8, 1.0
m/s and start windows 0.5, 1, 2 s. The original weight fails all three 1.0 m/s
cases (acceleration squared residual approximately 0.15153 > 0.05). With
penna_ts=1000 all nine cases solve (acceleration residual approximately
0.00122–0.00160 < 0.05).

Change only the replanner backup start-delay penalty to 1000; preserve velocity,
acceleration, angular, thrust, corridor and watchdog limits. The regression test
compares original and configured weights and checks the true polynomial maximum
acceleration, initial velocity continuity, and terminal rest, in addition to the
optimizer acceptance log. It uses ROS_DOMAIN_ID=87 and visualization publishers
only. The separate planner configuration retains its original delay penalty.

This is controlled optimizer validation, not a claim that the user's map/goal has
been replayed or that all backup problems are feasible. Live SITL verification
remains necessary. Replanner failure still deliberately stops command output and
flight_manager still latches hover after command timeout.
