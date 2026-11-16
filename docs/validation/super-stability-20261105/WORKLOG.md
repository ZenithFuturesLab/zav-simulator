# Active autonomous stability work (2026-11-05)

Goal is ACTIVE: user requests continuous SITL test/fix until stable, notify only
when evidence supports stability. Do not mark complete based on unit tests alone.
No subagents allowed. All flight commands in this run are authorized SITL tests.

Live system started locally (ROS_DOMAIN_ID=119 inherited env; .zav.env=0 but
simulator/environment.sh preserves caller domain). Native GPU/display :1.
- Simulator launcher exec session 64949; tmux zav_sim, PX4 pane 0, Isaac pane 1.
- Latest FAST-LIVO supervisor exec session 74253, output /tmp/zav_stability_fastlivo.log.
- Latest replanner session 64108, output /tmp/zav_stability_replanner.log.
- Latest autonomous flight: session 83387, recovery-07.jsonl, output
  /tmp/zav_stability_flight.log. It auto-lands and waits for disarm on success/failure.
  ALWAYS check process/current log before starting another manager.

New instrumented driver: simulator/tools/runtime/check_super_tracking.py
Requires --sitl and checks local PX4 and simulator processes, refuses duplicate
setpoint publishers; uses real flight-manager methods T/2/L. Records JSONL source
IMU, propagated/mapped LIO, reference commands, baro source status, 20Hz aggregate
state including measured-time aligned and raw errors. Always normal lands; no kill.
Options --offsets 'dx,dy;...', --fail-replans N, --inject-on-goal I,
--inject-when-backup, --require-recovery. Replanner must explicitly launch with
`enable_sitl_test_hooks:=true` for failure injection (default false/read-only).

Changes during this work:
1. Correct epoch alignment: manager predicts delayed LIO position to command time
   using validated child velocity rotated into world (max age stays 0.3 s), keeps
   0.5m limit. Replanner compares trajectory at actual odom measurement time.
2. ROGMap ROS2 odom now updates world velocity; it was never set, uninitialized.
   RobotState shared in new rog_map/include/super_utils/robot_state.hpp because
   two existing type_utils headers shared guards and duplicated the struct.
   Both now include one definition. Body twist positive-covariance contract
   enforced by rog_map_ros/odometry_velocity.hpp; unknown velocity becomes NaN.
3. FAST-LIVO imu propagation previously integrated newest acceleration over all
   callback backlog dt, exaggerating brief contact impulses. New helper
   imu_propagation_buffer.hpp integrates EVERY buffered IMU at its own interval,
   supports EKF state reset replay, does not publish state newer than input.
   Actual landing LIO jump disappeared after this correction.
4. Manager publishes retained Bool /zav_super/execution_enabled at 5Hz and changes;
   requires armed+flying+mode2+setpoints allowed+not blocked/LIO-latched. Replanner
   requires fresh true enable, drops goals when disabled, cancels old mission on
   landing/manual/blocked. Unit test topic remaps include this new Bool channel.
5. Opt-in C++ SITL failure-budget hook returns ordinary FAILED without changing
   committed snapshot. Finite budget auto-decrements. Normal launch is unaffected.
6. Safety status diagnostics now include baro parameter and aid receive ages.
   No baro/timeout/error safety thresholds have been relaxed.

Test records:
- baseline-01: 4 x 1m goals reached, raw max .451m / epoch aligned max .301m.
  Original driver said PASS but contact caused LIO safety position jump .78m;
  stricter driver now rejects landing latches. baseline summary explicitly false.
- aligned-02 (after epoch/velocity/IMU changes): 4 x 2m goals + normal landing PASS,
  raw max .519m, aligned max .318m, no tracking or LIO latch; plan_count exactly4.
- aligned-03: 3 x 3m goals reached; last goal stopped on LIO safety READY false.
  Cause was baro aid freshness >1s, state WAIT_BARO_OFF for ~.26s, no motion jump.
  Kept fail-closed behavior. Normal landing completed without LIO jump.
  Predicted error max .264m / aligned max .253m. Need investigate/observe telemetry.
- recovery-04: INVALID trial: old retained planner halt status immediately triggered
  tester land before new goal ack. Fixed tester epoch/grace handling. No valid flight.
- recovery-05: 8,4 out-and-back (~18m), 8 forced ordinary failures, real stop then
  rest-plan auto restart, both goals reached and normal land. Verification FAIL
  because initial trajectory had no backup; must not claim actual backup coverage.
- recovery-06: same out/back, 10 failures on second initial goal; both goals + normal
  land, also verification FAIL backup coverage. Backup existed later in successful
  replans but injection occurred too early, before backup existed.
- recovery-07 CURRENT: waits until status backup=GENERATED then injects 10 failures.
  Must prove EXECUTING_BACKUP, stop + PlanFromRest automatic restart, complete goals,
  normal land, no manager/safety latch. Output /tmp/zav_stability_flight.log.

Baro raw telemetry in 04 and05 was ~10Hz, max gap .114/.107s. The rare >1s gap in03
was not yet captured in raw baro recording (recording added after03). Current strict
READY logic should still stop for genuine stale proof; do not waive/ignore it.

Validation passed after current changes: C++ zav_super CTest 3/3; FAST-LIVO test
imu_propagation_buffer 1/1; Python tracking/control-timer/safety ROS 24 passed
(before most recent new pure Bool handshake test; rerun that set once before final).
Build logs: /tmp/zav_stability_build.log, /tmp/zav_stability_livo_build.log.

Remaining acceptance work:
- Actual committed BACKUP recovery proof (07 in progress).
- At least another nominal multi-goal/long path repeat after current fixes.
- Test planner/command interruption and LIO safety jump automatic landing using
  safe shadow fault relay (existing safety_experiment:=True + check_autoland.py),
  preserving singleton manager. Instrument and ensure ground/disarmed each time.
- Investigate repeated telemetry gap if it recurs. Do not claim all stable if
  unexplained safety/track latch persists in nominal trials.
- Review fixes, run required tests, save summaries/report with bounded SITL claims.
- Stop test processes safely and leave aircraft disarmed. Goal complete only once
  acceptance supported; then notify user in Korean with concise results and limits.

FINAL UPDATE: acceptance criteria passed after nominal trials normal-13/14 and
height-15 (10/10 goals) and recovery-07 + safety scenarios 09/10/11/12. See README,
acceptance.json and final-ground-state.json. Added consistent mutex snapshots for
ROG-Map pose/velocity/time before final normal trials. Python 25/25, C++ SUPER
3/3, IMU propagation 1/1 passed. Test flights ended landed/disarmed, normal baro OFF
confirmed and LIO READY/not latched. Test processes were sent graceful stop signals.
All earlier unsuccessful/invalid trials remain in the report and raw artifacts.
