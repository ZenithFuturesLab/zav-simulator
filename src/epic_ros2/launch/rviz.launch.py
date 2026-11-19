"""Display existing EPIC markers without changing the planner or controller."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('epic_ros2'), 'config', 'epic.rviz')
    frames = Node(
        package='tf2_ros', executable='static_transform_publisher', name='epic_rviz_frames',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0',
                   '--yaw', '0', '--frame-id', 'camera_init', '--child-frame-id', 'world'],
        parameters=[{'use_sim_time': True}], output='screen')
    rviz = Node(
        package='rviz2', executable='rviz2', name='epic_rviz', arguments=['-d', config],
        parameters=[{'use_sim_time': True}], output='screen')
    return LaunchDescription([
        frames, rviz,
        RegisterEventHandler(OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason='EPIC RViz closed'))]))])
