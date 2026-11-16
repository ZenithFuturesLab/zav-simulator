"""SUPER + LIO adapter + ROS 2 OMMPC. Isaac runs in its own process."""
import os
from pathlib import Path
import sys
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, EmitEvent, LogInfo, ExecuteProcess
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup(context):
    share=Path(get_package_share_directory('zav_super'))
    workspace=Path(os.environ['ZAV_WS_DIR'])
    scenario=LaunchConfiguration('scenario').perform(context)
    backend=LaunchConfiguration('backend').perform(context)
    rviz=LaunchConfiguration('rviz').perform(context).lower() in ('true','1')
    execute=LaunchConfiguration('execute').perform(context).lower() in ('true','1')
    profile=LaunchConfiguration('profile').perform(context)
    common_config=LaunchConfiguration('common_config').perform(context)
    if scenario=='mixed' and profile!='common':
        raise RuntimeError('mixed requires profile:=common')
    if common_config and profile!='common':
        raise RuntimeError('common_config requires profile:=common')
    if profile=='common' and backend!='super_lio':
        raise RuntimeError('common profile currently supports only super_lio')
    # Physical GUI tuning is specific to this scene and estimator.
    planner_config=share/'config'/('high_speed_super_lio.yaml'
        if scenario=='high_speed' and backend=='super_lio' else f'{scenario}.yaml')
    controller_config=share/'config'/f'ommpc_{scenario}.yaml'
    if profile=='common':
        planner_config=controller_config=(Path(common_config).expanduser().resolve()
            if common_config else share/'config'/'common_super_lio.yaml')
        if not planner_config.is_file():
            raise RuntimeError(f'Configuration file not found: {planner_config}')
    controller_parameters=(yaml.safe_load(controller_config.read_text())['zav_ommpc_px4']['ros__parameters']
                           if profile=='common' else str(controller_config))
    sys.path.insert(0,str(workspace/'simulator/pegasus/simulator/logic/sensors'))
    from fastlivo_calibration import load_calibration
    calibration=load_calibration(os.environ.get('PEGASUS_FASTLIVO_CONFIG_DIR',str(workspace/'src/FAST-LIVO2/config')),[.01215,0.,.11503],[0.,22.,0.])
    # Reuse exactly the existing mapper, external vision bridge and jump safety.
    lio=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(workspace/'simulator/tools/rviz/fastlivo_sim.launch.py')),
        launch_arguments={'backend':backend,
                          'navigation_profile':'true',
                          'navigation_config':str(planner_config) if profile=='common' else '',
                          'camera':'True' if backend=='fastlivo' else 'False','rviz':'False','safety':'True'}.items())
    def critical(): return [EmitEvent(event=Shutdown(reason='Critical SUPER navigation node exited'))]
    actions=[LogInfo(msg=f'SUPER planner config: {planner_config}'),lio,
        Node(package='zav_super',executable='lio_adapter.py',parameters=[{
            'scenario':scenario,'imu_position':calibration['imu_position'],'imu_orientation':calibration['imu_orientation']}],output='screen',on_exit=critical()),
        Node(package='super_planner',executable='fsm_node',parameters=[{
            'use_sim_time':True,'zav_mission_gate':True,'config_name':str(planner_config)}],output='screen',on_exit=critical()),
        Node(package='zav_super',executable='ommpc_px4',parameters=[{'use_sim_time':True},controller_parameters],output='screen',on_exit=critical()),
        Node(package='zav_super',executable='benchmark_mission.py',parameters=[{'use_sim_time':True,'execute':execute}],output='screen')]
    if rviz: actions.append(Node(package='rviz2',executable='rviz2',prefix=['bash '+str(workspace/'simulator/tools/rviz/gui_exec.sh')],arguments=['-d',str(share/'rviz/super.rviz')],parameters=[{'use_sim_time':True}],output='screen'))
    telemetry_dir=LaunchConfiguration('telemetry_dir').perform(context)
    if telemetry_dir:
        actions.insert(0,ExecuteProcess(cmd=[sys.executable,str(workspace/'simulator/tools/super/flight_telemetry.py'),
            '--output',telemetry_dir,'--config',str(planner_config),'--scenario',scenario,'--backend',backend],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario',default_value='high_speed',choices=['high_speed','dense','mixed']),
        DeclareLaunchArgument('backend',default_value='super_lio',choices=['super_lio','fastlivo','fastlio']),
        DeclareLaunchArgument('profile',default_value='legacy',choices=['legacy','common']),
        DeclareLaunchArgument('common_config',default_value='',
                             description='Optional shared YAML path for isolated common-profile experiments'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('execute',default_value='false'),
        DeclareLaunchArgument('telemetry_dir',default_value=''),
        OpaqueFunction(function=setup)])
