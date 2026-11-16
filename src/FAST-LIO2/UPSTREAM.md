Source: https://github.com/hku-mars/FAST_LIO
Branch: ROS2
Commit: cc3c9e8b1469b33b8072764d6db2348ed6dc597f

Imported only estimator/preprocessing, required headers (including upstream
ikd-Tree submodule), Pose6D interface and license. No original media, example
maps, launchers or viewers. Config uses existing ZAV Mid360 calibration.

Local changes: C++17; compatible sensor QoS; body-frame velocity/angular velocity
in odometry; covariance filled before publishing; mounted-IMU frame contract;
opt-in zav_compat gravity alignment and high-rate IMU predictor. The predictor
uses corrected ESKF state/bias/gravity, replays buffered IMU after corrections
and preserves measurement timestamps. Scan matching/ESKF/map update remains upstream.
