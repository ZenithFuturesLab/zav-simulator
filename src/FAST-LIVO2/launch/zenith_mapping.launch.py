import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()
    use_rviz = LaunchConfiguration('rviz')
    rviz_launch_arg = DeclareLaunchArgument('rviz', default_value='True')
    ld.add_action(rviz_launch_arg)

    use_camera = LaunchConfiguration('camera')
    camera_launch_arg = DeclareLaunchArgument(
        'camera',
        default_value='True',
        description='Enable camera (True for LIVO RGB, False for pure LIO LiDAR+IMU)'
    )
    ld.add_action(camera_launch_arg)

    fast_livo_share = get_package_share_directory('fast_livo')
    livo_params_file = os.path.join(fast_livo_share, 'config', 'zenith.yaml')
    camera_params_file = os.path.join(fast_livo_share, 'config', 'zenith_camera_pinhole.yaml')
    rviz_config_file = os.path.join(fast_livo_share, 'rviz_cfg', 'fast_livo2.rviz')

    img_en_param = ParameterValue(
        PythonExpression(['1 if "', use_camera, '" in ["True", "true", "1"] else 0']),
        value_type=int
    )

    livo = Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        name='laserMapping',
        parameters=[
            livo_params_file,
            camera_params_file,
            {'common.img_en': img_en_param}
        ],
        output="screen"
    )
    ld.add_action(livo)

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output="screen"
    )
    ld.add_action(rviz_cmd)
    return ld

