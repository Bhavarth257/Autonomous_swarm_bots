"""Launch actions for one robot's autonomy stack: SLAM, traversability, Nav2, explorer."""
import os

from launch.actions import TimerAction
from launch_ros.actions import Node

from rescue_navigation import nav_config
from rescue_navigation.nav2_bringup import NAV2_NODES
from rescue_navigation.robot_profiles import DEFAULT, profile

NAV2_PACKAGES = {'controller_server': 'nav2_controller', 'planner_server': 'nav2_planner',
                 'behavior_server': 'nav2_behaviors', 'bt_navigator': 'nav2_bt_navigator',
                 'velocity_smoother': 'nav2_velocity_smoother'}
# controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel (the gait plugin)
REMAPS = {'controller_server': [('cmd_vel', 'cmd_vel_nav')],
          'velocity_smoother': [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]}
assert list(NAV2_PACKAGES) == NAV2_NODES


def robot_autonomy(ns, building_rect, map_in_world, auto_start=False, out_dir=None,
                   time_limit=1200.0, startup_delay=5.0, robot_starts=(), robot=DEFAULT, team_size=1):
    """robot_starts: world (x, y) of every robot's start, so none becomes a map obstacle."""
    """building_rect: (x0, y0, x1, y1) world; map_in_world: robot start pose (x, y, yaw)."""
    out_dir = out_dir or os.path.expanduser('~/.ros/rescue_worlds/params')
    params = nav_config.write(ns, out_dir, robot)
    prof = profile(robot)
    sim = {'use_sim_time': True}
    actions = [
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             namespace=ns, parameters=[params['slam']], output='log'),
        Node(package='rescue_navigation', executable='traversability', namespace=ns,
             parameters=[{'building_rect': [float(v) for v in building_rect],
                          'map_in_world': [float(v) for v in map_in_world],
                          'robot_starts': [float(v) for p in robot_starts for v in p] or [0.0, 0.0],
                          **{k: float(prof[k]) for k in ('body_height', 'step_max', 'blind_radius', 'self_radius',
                                                        'body_radius')},
                          'robot_clearance': float(prof['robot_clearance']), **sim}],
             output='log'),
        Node(package='rescue_navigation', executable='explorer', namespace=ns, output='screen',
             parameters=[{'building_rect': [float(v) for v in building_rect],
                          'map_in_world': [float(v) for v in map_in_world],
                          'auto_start': auto_start, 'time_limit': float(time_limit),
                          'holonomic': prof['holonomic'], 'return_speed': float(prof['return_speed']),
                          'robot_radius': float(prof['reach_radius']), 'team_size': int(team_size), **sim}]),
    ]
    for name, pkg in NAV2_PACKAGES.items():
        actions.append(Node(package=pkg, executable=name, name=name, namespace=ns,
                            parameters=[params['nav2']], remappings=REMAPS.get(name, []), output='log'))
    # No lifecycle_manager: the explorer configures and activates Nav2 itself, retrying
    # (nav2_bringup). Each robot's stack starts a few seconds after the previous one, once
    # Gazebo is up: everything starting in the same second lost service calls and TF and
    # left a robot without SLAM or Nav2 for the whole mission.
    return [TimerAction(period=startup_delay, actions=actions)]
