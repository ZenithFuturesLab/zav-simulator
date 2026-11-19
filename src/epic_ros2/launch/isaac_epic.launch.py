"""EPIC exploration and trajectory server connected to the ZAV LIO/PX4 pipeline."""
import os
import shutil
import tempfile
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup(context):
    config = LaunchConfiguration('config').perform(context)
    external = LaunchConfiguration('external_local_planner').perform(context).lower() == 'true'
    calibration_dir = os.environ.get('PEGASUS_FASTLIVO_CONFIG_DIR',
        os.path.join(get_package_share_directory('fast_livo'), 'config'))
    with open(os.path.join(calibration_dir, 'zenith.yaml')) as stream:
        calibration = yaml.safe_load(stream)['/laserMapping']['ros__parameters']['extrin_calib']
    rotation = Rotation.from_euler('ZYX', [0., 22., 0.], degrees=True) * Rotation.from_matrix(
        np.array(calibration['extrinsic_R']).reshape(3, 3)).inv()
    mount = (np.array([.01215, 0., .11503]) - rotation.apply(calibration['extrinsic_T'])).tolist()
    tsp = tempfile.mkdtemp(prefix='zav-epic-tsp-')
    remaps = [('/planning/' + name, '/epic/' + name) for name in
        ['trajectory', 'yaw_trajectory', 'heartbeat', 'replan', 'new', 'static', 'state']]
    remaps += [('/planning/land_requested', '/zav/epic/land_requested'), ('/position_cmd', '/epic/position_cmd'),
        ('/waypoint_generator/waypoints', '/epic/trigger'),
        ('/move_base_simple/goal', '/epic/goal'), ('/quad_0/lidar_slam/odom', '/epic/odom')]

    nodes = [
        Node(package='epic_ros2', executable='exploration_node', name='epic_exploration',
            parameters=[config, {'exploration.tsp_dir': tsp, 'fsm.external_local_planner': external},
                *([{'odometry_topic': LaunchConfiguration('external_odometry_topic').perform(context),
                    'cloud_topic': LaunchConfiguration('external_cloud_topic').perform(context),
                    'lidar_perception.odom_twist_in_world': True}] if external else [])], remappings=remaps,
            output='screen'),
        Node(package='epic_ros2', executable='traj_server', name='epic_traj_server',
            parameters=[config], remappings=remaps, output='screen'),
        Node(package='epic_ros2', executable='isaac_adapter.py', name='epic_isaac_adapter',
            parameters=[{'trajectory_feedforward': LaunchConfiguration('feedforward').perform(context).lower() == 'true',
                'use_sim_time': False, 'imu_mount_position': mount,
                'imu_mount_quaternion': rotation.as_quat().tolist()}],
            output='screen')]
    if external:
        # SUPER supplies the common clock/body odometry and owns all local
        # commands. Do not start EPIC's trajectory server or command adapter.
        nodes = nodes[:1]
    remaining = set(nodes)
    stopping = False

    def exited(event, context):
        nonlocal stopping
        remaining.discard(event.action)
        if not remaining:
            shutil.rmtree(tsp, ignore_errors=True)
        if not stopping:
            stopping = True
            return [EmitEvent(event=Shutdown(reason='EPIC node exited'))]
        return []

    return nodes + [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=exited)) for node in nodes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('feedforward', default_value='false'),
        DeclareLaunchArgument('external_local_planner', default_value='false'),
        DeclareLaunchArgument('external_odometry_topic', default_value='/zav/super/odom'),
        DeclareLaunchArgument('external_cloud_topic', default_value='/zav/super/cloud_world'),
        DeclareLaunchArgument('config', default_value=os.path.join(
            get_package_share_directory('epic_ros2'), 'config', 'isaac_warehouse.yaml')),
        OpaqueFunction(function=setup)])
