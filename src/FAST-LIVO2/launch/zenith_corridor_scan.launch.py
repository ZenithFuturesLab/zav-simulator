import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Launch RViz2 (Default: False for mobile/headless scan)'
    )
    ld.add_action(rviz_arg)

    # 1. Camera Node (Hikrobot MV-CU013, 10Hz Free-run)
    mvs_share = get_package_share_directory('mvs_ros2_driver')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mvs_share, 'launch', 'zenith_camera.launch.py')
        )
    )
    ld.add_action(camera_launch)

    # 2. LiDAR Node (Livox Mid-360S, CustomMsg 10Hz)
    livox_share = get_package_share_directory('livox_ros_driver2')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, 'launch_ROS2', 'msg_MID360s_launch.py')
        )
    )
    ld.add_action(lidar_launch)

    # 3. FAST-LIVO2 Mapping Node
    fast_livo_share = get_package_share_directory('fast_livo')
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_livo_share, 'launch', 'zenith_mapping.launch.py')
        ),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items()
    )
    ld.add_action(mapping_launch)

    # 4. Real-time Sensor Health Monitor (C++ Zero-Copy 200Hz Watchdog)
    from launch.actions import ExecuteProcess
    health_monitor = ExecuteProcess(
        cmd=['/bin/bash', '-c', 'source /opt/ros/humble/setup.bash && source /root/zenith_ws/install/setup.bash && exec /root/zenith_ws/install/mvs_ros2_driver/lib/mvs_ros2_driver/zenith_health_monitor'],
        output='screen'
    )
    ld.add_action(health_monitor)

    return ld
