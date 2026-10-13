"""Launch actions for one robot's autonomy stack: SLAM, traversability, Nav2, explorer."""
import os

from launch.actions import TimerAction
from launch_ros.actions import Node

from rescue_navigation import nav_config

NAV2_PACKAGES = {'controller_server': 'nav2_controller', 'planner_server': 'nav2_planner',
                 'behavior_server': 'nav2_behaviors', 'bt_navigator': 'nav2_bt_navigator',
                 'velocity_smoother': 'nav2_velocity_smoother'}
# controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel (the gait plugin)
REMAPS = {'controller_server': [('cmd_vel', 'cmd_vel_nav')],
          'velocity_smoother': [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]}
NAV2_NODES = list(NAV2_PACKAGES)


def robot_autonomy(ns, building_rect, map_in_world, auto_start=False, out_dir=None,
                   time_limit=1200.0, startup_delay=5.0, robot_starts=()):
    """robot_starts: world (x, y) of every robot's start, so none becomes a map obstacle."""
    """building_rect: (x0, y0, x1, y1) world; map_in_world: robot start pose (x, y, yaw)."""
    out_dir = out_dir or os.path.expanduser('~/.ros/rescue_worlds/params')
    params = nav_config.write(ns, out_dir)
    sim = {'use_sim_time': True}
    actions = [
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             namespace=ns, parameters=[params['slam']], output='log'),
        Node(package='rescue_navigation', executable='traversability', namespace=ns,
             parameters=[{'building_rect': [float(v) for v in building_rect],
                          'map_in_world': [float(v) for v in map_in_world],
                          'robot_starts': [float(v) for p in robot_starts for v in p] or [0.0, 0.0], **sim}],
             output='log'),
        Node(package='rescue_navigation', executable='explorer', namespace=ns, output='screen',
             parameters=[{'building_rect': [float(v) for v in building_rect],
                          'map_in_world': [float(v) for v in map_in_world],
                          'auto_start': auto_start, 'time_limit': float(time_limit), **sim}]),
    ]
    for name, pkg in NAV2_PACKAGES.items():
        actions.append(Node(package=pkg, executable=name, name=name, namespace=ns,
                            parameters=[params['nav2']], remappings=REMAPS.get(name, []), output='log'))
    # Bring Nav2 up only once its nodes exist and the machine has settled: with three
    # robots starting at once a slow-to-configure bt_navigator times the manager out.
    actions.append(TimerAction(period=startup_delay, actions=[
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', namespace=ns, output='log',
             parameters=[{'autostart': True, 'node_names': NAV2_NODES, 'bond_timeout': 10.0, **sim}])]))
    return actions
