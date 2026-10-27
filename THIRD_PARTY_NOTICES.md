# Third-party notices

This repository combines project-specific simulation changes with vendored source. There is no blanket license overriding upstream terms. Keep existing copyright headers and license files when redistributing.

| Component | Source / terms |
| --- | --- |
| FAST-LIVO2 | https://github.com/hku-mars/FAST-LIVO2 — vendored LICENSE is GNU GPL version 2. The copied package.xml previously said BSD; it is corrected to match the supplied LICENSE. |
| livox_ros_driver2 | https://github.com/Livox-SDK/livox_ros_driver2 — MIT; see src/livox_ros_driver2/LICENSE.txt. |
| px4_msgs | https://github.com/PX4/px4_msgs — BSD 3-Clause; see src/px4_msgs/LICENSE. Message definitions match the locked PX4 revision. |
| Sophus | Hauke Strasdat and contributors; MIT notices are retained in the vendored headers. |
| rpg_vikit | https://github.com/uzh-rpg/rpg_vikit and ROS 2 port contributors; existing attribution retained. The supplied ROS 2 port has incomplete package license metadata; verify upstream redistribution terms before wider distribution. |
| PX4-Autopilot | https://github.com/PX4/PX4-Autopilot — BSD 3-Clause and component-specific terms; fetched at the locked commit. |
| PegasusSimulator | https://github.com/PegasusSimulator/PegasusSimulator — BSD 3-Clause and asset-specific terms; source and assets vendored in simulator/pegasus, revision recorded in UPSTREAM.json. LICENSE and original copyright headers retained. |
| Micro XRCE-DDS Agent | https://github.com/eProsima/Micro-XRCE-DDS-Agent — Apache 2.0 and dependency-specific terms; fetched at the locked commit. |
| Isaac Sim | NVIDIA proprietary distribution; installed separately under NVIDIA terms. No Isaac binaries or installed environments are redistributed here. |

The ZAV USD and project configuration are included for this private project. No new license grant for those assets is asserted by these installation scripts. Keep the repository private until the organization decides its distribution policy.
