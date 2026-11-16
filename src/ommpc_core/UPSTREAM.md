Source: https://github.com/oliverwu515/ommpc_controller
Branch: refact-decouple
Commit: c421d2fd1f0b799b31657d6c6a5f7f8191c38a09

Only ommpc_core is imported. MAVROS/ROS1 wrappers, traj_utils, text examples,
dynamic reconfigure and media are not imported. The existing upstream
ROS-independent solver/dynamics implement OMMPC; this is not a replacement PD.

Local changes: ament CMake build, exported OSQP 0.6.3/Eigen dependency,
OSQP CSC-descriptor cleanup and invalid-call guard, SUPER yaw polynomial
sampling (the upstream TODO), yaw-time offset, wrapped-angle handling and
local yaw reference governor for discontinuous goal replans, and the separate PX4 ROS2 adapter in zav_super.
The original package license is TODO; no blanket license is asserted here.
polynomial_trajectory.h carries its own MIT notice, which is retained.
