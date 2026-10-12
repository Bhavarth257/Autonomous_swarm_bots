"""Full autonomous mission.

Per robot: SLAM, traversability, Nav2, explorer, swarm agent. Shared: Gazebo, the
radio channel and the victim detector (one process for the GPU, separate outputs).

  ros2 launch rescue_bringup mission.launch.py seed:=4 num_robots:=3
  -> press START in the mission dashboard (or: ros2 service call /r1/mission/start std_srvs/srv/Trigger)
  ros2 topic echo /r1/team/state                                 # what r1 knows

auto_start:=true starts exploring as soon as Nav2 is up.
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rescue_navigation.launch_helpers import robot_autonomy
from rescue_world.generate import DEFAULT_OUT, generate


def _autonomy(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    seed, n = int(arg('seed')), int(arg('num_robots'))
    # Same seed -> same files the sim launch generated; we only need the ground plan
    # the rescuers would have: the building outline and where the robots start.
    paths = generate(seed=seed, out_dir=DEFAULT_OUT, n_robots=max(n, 3))
    with open(paths['json']) as f:
        gt = json.load(f)
    rect = (0.0, 0.0, gt['building']['width'], gt['building']['height'])
    sim = {'use_sim_time': True}
    names = [gt['spawns'][k]['name'] for k in range(n)]
    actions = []
    for k in range(n):
        sp = gt['spawns'][k]
        start = (sp['x'], sp['y'], sp['yaw'])
        actions += robot_autonomy(sp['name'], rect, start,
                                  auto_start=arg('auto_start') == 'true',
                                  time_limit=float(arg('time_limit')), startup_delay=5.0 + 4.0 * k)
        if arg('swarm') == 'true':
            actions.append(Node(package='rescue_swarm', executable='swarm_agent', namespace=sp['name'],
                                output='log', parameters=[{'building_rect': list(rect),
                                                           'map_in_world': [float(v) for v in start], **sim}]))
    if arg('swarm') == 'true':
        actions.append(Node(package='rescue_swarm', executable='radio', output='log',
                            parameters=[{'robots': names, 'map_yaml': paths['map_yaml'],
                                         'los_range': float(arg('radio_range')),
                                         'wall_range': float(arg('radio_wall_range')), **sim}]))
    if arg('perception') == 'true':
        poses = [float(v) for k in range(n)
                 for v in (gt['spawns'][k]['x'], gt['spawns'][k]['y'], gt['spawns'][k]['yaw'])]
        actions.append(Node(package='rescue_perception', executable='victim_detector', output='screen',
                            parameters=[{'robots': names, 'maps_in_world': poses, **sim}]))
    if arg('evaluate') == 'true':
        out = os.path.expanduser(f"~/rescue_reports/seed{seed}_{n}robots")
        ev = Node(package='rescue_eval', executable='evaluate', output='screen',
                  parameters=[{'seed': seed, 'robots': names, 'duration': float(arg('time_limit')) + 240.0,
                               'out': out, **sim}])
        actions += [ev, RegisterEventHandler(OnProcessExit(target_action=ev,
                                                           on_exit=[EmitEvent(event=Shutdown(reason='evaluated'))]))]
    if arg('dashboard') == 'true':
        actions.append(Node(package='rescue_dashboard', executable='dashboard', output='log',
                            parameters=[{'seed': seed, 'robots': names,
                                         'snapshot': arg('dashboard_snapshot'), **sim}]))
    return actions


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rescue_bringup'), 'launch', 'sim.launch.py')),
        launch_arguments={'seed': LaunchConfiguration('seed'),
                          'odom_drift': LaunchConfiguration('odom_drift'),
                          'num_robots': LaunchConfiguration('num_robots'),
                          'gui': LaunchConfiguration('gui'),
                          'rviz': LaunchConfiguration('rviz'),
                          'slam': 'true'}.items())
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('num_robots', default_value='1'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('odom_drift', default_value='true'),
        DeclareLaunchArgument('time_limit', default_value='1200', description='seconds'),
        DeclareLaunchArgument('dashboard', default_value='true', description='mission control window'),
        DeclareLaunchArgument('evaluate', default_value='false',
                              description='score against ground truth, then shut down (batch runs)'),
        DeclareLaunchArgument('dashboard_snapshot', default_value='', description='save screenshots here'),
        DeclareLaunchArgument('perception', default_value='true', description='victim detector'),
        DeclareLaunchArgument('swarm', default_value='true', description='radio + swarm agents'),
        DeclareLaunchArgument('radio_range', default_value='12.0', description='line-of-sight range, m'),
        DeclareLaunchArgument('radio_wall_range', default_value='4.0', description='range through walls, m'),
        sim,
        OpaqueFunction(function=_autonomy),
    ])
