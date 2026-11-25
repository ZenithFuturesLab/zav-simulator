"""EPIC exploration and trajectory server connected to the ZAV LIO/PX4 pipeline."""
import os
import math
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
    scene_config = LaunchConfiguration('scene_config').perform(context)
    ceiling = LaunchConfiguration('survey_max_z').perform(context)
    limits = []
    if ceiling:
        z = float(ceiling)
        if not math.isfinite(z):
            raise ValueError('The survey ceiling must be a finite world altitude in metres')
        limits.append({'flight_height.max_z': z, 'FrontierManager.target_max_z': z})
    flight_ceiling = LaunchConfiguration('survey_flight_max_z').perform(context)
    if flight_ceiling:
        flight_z = float(flight_ceiling)
        if not math.isfinite(flight_z) or (ceiling and flight_z > float(ceiling)):
            raise ValueError('Flight ceiling must be finite and within the survey volume')
        limits.append({'flight_height.max_z': flight_z})
    external = LaunchConfiguration('external_local_planner').perform(context).lower() == 'true'
    aircraft_parameters = {}
    if scene_config:
        with open(scene_config) as stream:
            profile = yaml.safe_load(stream)['/**']['ros__parameters']
        aircraft_parameters = {key: profile[key] for key in
            ('lidar_perception.lidar_pitch', 'lidar_perception.sensor_offset_body', 'VehicleMass')
            if key in profile}

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
            # Idle planning workers must yield CPU time to LIO and control.
            additional_env={'OMP_WAIT_POLICY': 'PASSIVE'},
            # Inherit the background priority in every planning/DDS worker,
            # including threads created after startup. Local control stays at
            # its original priority in the external-planner arrangement.
            prefix='nice -n 10' if external else '',
            parameters=[{'use_sim_time': True}, config, aircraft_parameters, *limits,
                {'exploration.tsp_dir': tsp, 'fsm.external_local_planner': external},
                *([{'odometry_topic': LaunchConfiguration('external_odometry_topic').perform(context),
                    'cloud_topic': LaunchConfiguration('external_cloud_topic').perform(context),
                    'exploration.state_reference_imu_topic': LaunchConfiguration('external_reference_imu_topic').perform(context),
                    'lidar_perception.odom_twist_in_world': True}] if external else [])], remappings=remaps,
            output='screen'),
        Node(package='epic_ros2', executable='traj_server', name='epic_traj_server',
            parameters=[{'use_sim_time': True}, config], remappings=remaps, output='screen'),
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
        DeclareLaunchArgument('external_reference_imu_topic', default_value='/drone0/mid360/imu',
            description='Raw timestamp reference independent of the body-odometry adapter; empty uses ROS clock'),
        DeclareLaunchArgument('survey_flight_max_z', default_value='', description='Optional ceiling for viewpoints and connectors within the survey volume'),
        DeclareLaunchArgument('survey_max_z', default_value='', description='Optional upper altitude of the survey volume in metres'),
        DeclareLaunchArgument('scene_config', default_value='',
            description='Optional LiDAR mounting and mass parameters from the active aircraft profile'),
        DeclareLaunchArgument('config', default_value=os.path.join(
            get_package_share_directory('epic_ros2'), 'config', 'isaac_warehouse.yaml'),
            description='Complete EPIC parameter file for the selected scene'),
        OpaqueFunction(function=setup)])
