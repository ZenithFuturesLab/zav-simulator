"""Launch FAST-LIVO2 and its mounted-IMU PX4 external odometry bridge."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    directory = Path(__file__).resolve().parent
    share = Path(get_package_share_directory("fast_livo"))
    # [ZAV 변경] 고정 TCP peer는 WSL 전용. Ubuntu 기본 DDS에는 강제하지 않습니다.
    tcp = os.environ.get("ZAV_ROS_TRANSPORT") == "cyclone_tcp"
    mapper_env = {"CYCLONEDDS_URI": (directory / "cyclonedds_fastlivo.xml").as_uri()} if tcp else {}
    viewer_env = {"CYCLONEDDS_URI": (directory / "cyclonedds_local.xml").as_uri()} if tcp else {}
    bridge_env = {"CYCLONEDDS_URI": (directory / "cyclonedds_bridge.xml").as_uri()} if tcp else {}
    camera = LaunchConfiguration("camera")
    image_enabled = ParameterValue(PythonExpression([
        '1 if "', camera, '" in ["True", "true", "1"] else 0'
    ]), value_type=int)

    def bridge(context):
        arguments = ['--publish', '--assume-enu-flu']
        if IfCondition(LaunchConfiguration('safety')).evaluate(context):
            arguments += ['--safety']
            for option in ('position_slack', 'angle_slack_deg', 'prediction_horizon'):
                arguments += ['--safety-' + option.replace('_', '-'),
                              LaunchConfiguration('safety_' + option).perform(context)]
            if IfCondition(LaunchConfiguration('safety_experiment')).evaluate(context):
                arguments += ['--safety-odom-topic', '/zav/lio_safety/test/odom']
        return [Node(package='zav_control', executable='lio_px4_bridge',
                     name='zav_lio_px4_odom_bridge',
                     condition=IfCondition(LaunchConfiguration('px4_bridge')),
                     arguments=arguments, additional_env=bridge_env, output='screen')]

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="True"),
        DeclareLaunchArgument("camera", default_value="True"),
        DeclareLaunchArgument("px4_bridge", default_value="True",
                              description="Publish FAST-LIVO2 odometry to PX4"),
        DeclareLaunchArgument('safety', default_value='True',
                              description='Detect only position/attitude jumps and request latched NAV_LAND'),
        DeclareLaunchArgument('safety_experiment', default_value='False',
                              description='SITL only: monitor through fault-injection relay'),
        DeclareLaunchArgument('safety_position_slack', default_value='0.5'),
        DeclareLaunchArgument('safety_angle_slack_deg', default_value='20.0'),
        DeclareLaunchArgument('safety_prediction_horizon', default_value='0.25'),
        DeclareLaunchArgument('baro_param_connection', default_value='udpin:127.0.0.1:14030',
                              description='Reserved PX4 onboard-payload MAVLink parameter link'),
        Node(package="fast_livo", executable="fastlivo_mapping", name="laserMapping",
             parameters=[str(share / "config/zenith.yaml"), str(share / "config/zenith_camera_pinhole.yaml"),
                         # [ZAV 변경] 실센서 YAML을 수정하지 않고 시뮬레이터 입력 토픽만 덮어씁니다.
                         {"uav.imu_rate_odom": True, "common.img_en": image_enabled,
                          "common.img_topic": "/drone0/front_camera/color/image_raw",
                          "common.imu_topic": "/drone0/mid360/imu",
                          "common.lid_topic": "/drone0/mid360/lidar"}],
             additional_env=mapper_env,
             output="screen"),
        OpaqueFunction(function=bridge),
        ExecuteProcess(cmd=[str(directory.parents[2] / '.cache/px4-python/bin/python'),
                            str(directory.parent / 'runtime/baro_parameter_bridge.py'),
                            '--connection', LaunchConfiguration('baro_param_connection')],
                       condition=IfCondition(PythonExpression([
                           '"', LaunchConfiguration('safety'), '".lower() in ("true", "1") and "',
                           LaunchConfiguration('px4_bridge'), '".lower() in ("true", "1")'])),
                       additional_env=viewer_env, output='screen'),
        ExecuteProcess(cmd=['python3', str(directory.parent / 'runtime/lio_safety_experiment.py'),
                            'relay'],
                       condition=IfCondition(LaunchConfiguration('safety_experiment')),
                       additional_env=viewer_env, output='screen'),
        Node(package="rviz2", executable="rviz2", name="fastlivo_rviz",
             prefix=["bash " + str(directory / "gui_exec.sh")],
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", str(share / "rviz_cfg/fast_livo2.rviz")],
             additional_env=viewer_env, output="screen"),
    ])
