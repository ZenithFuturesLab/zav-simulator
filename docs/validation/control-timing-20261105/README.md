# Live inspection and flight-manager timer correction — 2026-11-05

Read-only inspection of the running SITL stack; no arm, land, setpoint or goal
commands were sent, and no running flight process was restarted.

Observed state:
- Replanner: TRACKING_DIVERGED_NEW_GOAL_REQUIRED; plan_count=3,
  replan_success_count=4, replan_failure_count=0, tracking_error=0.566m.
- First flight-manager fault in its log: tracking error exceeds limit at
  1793804366.696; later new goals were accepted by the planner.
- Armed, OFFBOARD (nav_state=14); LIO READY, latch=false.
- PX4 measured velocity near zero; setpoint velocity [0,0,0], holding position.
- No active /zav_super/tracking_command messages during inspection.

After three seconds of discovery warmup, a five-second observation measured:

| Topic | Wall rate | Maximum interarrival gap |
|---|---:|---:|
| /clock | 19.2Hz | 52.9ms |
| /fmu/out/vehicle_local_position | 95.4Hz | 42.6ms |
| /fmu/in/offboard_control_mode | 19.0Hz | 57.6ms |
| /fmu/in/trajectory_setpoint | 19.0Hz | 57.7ms |

The flight manager used a 20ms timer on the ROS node clock. With use_sim_time=true,
that timer was serviced only when the approximately 20Hz /clock advanced. This
is a confirmed scheduling defect relative to the intended 50Hz control rate.
The first tracking-error event predates this observation; its exact cause cannot
be proved from the available target/measurement logs. Do not claim that timer
correction alone proves the flight-tracking problem resolved.

Changes:
- Schedule the flight-manager control timer using STEADY_TIME at 50Hz.
  Keep the node ROS clock in simulation time for reference timestamp validation.
- Poll keyboard input without a second 5ms blocking wait after every ROS callback.
- Log target/measured ENU positions, Euclidean tracking error and command/LIO/PX4
  receive ages when tracking latches. Preserve all thresholds and manual recovery.

Validation:
- New ROS test publishes /clock at 20Hz, enables use_sim_time and verifies the
  control timer continues at the independent 50Hz schedule (with a jitter margin).
  All aircraft topics are remapped to a unique test prefix and the actual control
  callback is replaced by a counter, so the test cannot send aircraft commands.
- 20 Python tests passed, including tracking conversion, source timeout messages,
  LIO safety ROS integration and timer scheduling.
- zav_control build passed. The running user's manager retains its old timer
  until it is restarted; no in-flight hot replacement was attempted.

Next live validation: land, restart flight_manager with the same arguments,
T → settled hover → 2 → nearby goal; then observe setpoint rate and tracking error.
