Source: https://github.com/Liansheng-Wang/Super-LIO
Branch: ros2
Commit: f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2
Vendored packages basic and super_lio, with upstream GPL-3.0 license retained.

ZAV changes: Humble dependency discovery, explicit TBB linkage, optional mounted-IMU odometry compatibility and robust Livox packet validation, and replay of IMU history after delayed LiDAR corrections to keep high-rate odometry time-consistent. See docs/super-lio.md in the parent workspace.
