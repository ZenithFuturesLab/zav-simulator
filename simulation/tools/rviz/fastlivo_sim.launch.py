"""Launch FAST-LIVO2 and its mounted-IMU PX4 external odometry bridge."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="True"),
        DeclareLaunchArgument("camera", default_value="True"),
        DeclareLaunchArgument("px4_bridge", default_value="True",
                              description="Publish FAST-LIVO2 odometry to PX4"),
        Node(package="fast_livo", executable="fastlivo_mapping", name="laserMapping",
             parameters=[str(share / "config/zenith.yaml"), str(share / "config/zenith_camera_pinhole.yaml"),
                         # [ZAV 변경] 실센서 YAML을 수정하지 않고 시뮬레이터 입력 토픽만 덮어씁니다.
                         {"uav.imu_rate_odom": True, "common.img_en": image_enabled,
                          "common.img_topic": "/drone0/front_camera/color/image_raw",
                          "common.imu_topic": "/drone0/mid360/imu",
                          "common.lid_topic": "/drone0/mid360/lidar"}],
             additional_env=mapper_env,
             output="screen"),
        Node(package="zav_control", executable="lio_px4_bridge",
             name="zav_lio_px4_odom_bridge",
             condition=IfCondition(LaunchConfiguration("px4_bridge")),
             arguments=["--publish", "--assume-enu-flu"],
             additional_env=bridge_env, output="screen"),
        Node(package="rviz2", executable="rviz2", name="fastlivo_rviz",
             prefix=["bash " + str(directory / "gui_exec.sh")],
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", str(share / "rviz_cfg/fast_livo2.rviz")],
             additional_env=viewer_env, output="screen"),
    ])
