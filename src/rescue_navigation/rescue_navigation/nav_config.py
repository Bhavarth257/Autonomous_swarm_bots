"""Per-robot SLAM Toolbox and Nav2 parameters for a namespaced hexapod.

Starts from the installed defaults and overrides frames, topics, the costmap layers
(fused nav_map + live 2D scan) and the controller tuning for the walking gait.
Everything is nested under the robot namespace, so one file serves /rN/<node>.
"""
import copy
import os

import yaml
from ament_index_python.packages import get_package_share_directory

ROBOT_RADIUS = 0.22      # collision radius: the legs (span ~0.53 x 0.58 m) may brush door jambs
INFLATION = 0.7          # paths keep well clear of walls anyway
CRUISE = 0.35            # m/s, below the gait's 0.45 cap so turns stay smooth


def _load(pkg, *path):
    with open(os.path.join(get_package_share_directory(pkg), *path)) as f:
        return yaml.safe_load(f)


def slam_params(ns):
    p = _load('slam_toolbox', 'config', 'mapper_params_online_async.yaml')['slam_toolbox']['ros__parameters']
    p.update({
        'use_sim_time': True,
        'odom_frame': f'{ns}/odom',
        'map_frame': f'{ns}/map',
        'base_frame': f'{ns}/base_footprint',
        'scan_topic': f'/{ns}/scan',
        'map_name': f'/{ns}/map',          # Humble default is the absolute /map
        'mode': 'mapping',
        'resolution': 0.05,
        'max_laser_range': 12.0,
        'minimum_travel_distance': 0.2,
        'minimum_travel_heading': 0.2,
        'map_update_interval': 1.0,
        'transform_publish_period': 0.05,
        'tf_buffer_duration': 30.0,
        'transform_timeout': 0.3,
        'throttle_scans': 1,
        'do_loop_closing': True,
    })
    return {ns: {'slam_toolbox': {'ros__parameters': p}}}


def nav2_params(ns):
    d = _load('nav2_bringup', 'params', 'nav2_params.yaml')
    map_frame, odom_frame, base = f'{ns}/map', f'{ns}/odom', f'{ns}/base_footprint'
    nav_map = f'/{ns}/nav_map'

    bt = d['bt_navigator']['ros__parameters']
    bt.update({'global_frame': map_frame, 'robot_base_frame': base, 'odom_topic': f'/{ns}/odom'})

    ctrl = d['controller_server']['ros__parameters']
    ctrl['controller_frequency'] = 10.0
    ctrl['progress_checker'].update({'required_movement_radius': 0.3, 'movement_time_allowance': 20.0})
    ctrl['general_goal_checker'].update({'xy_goal_tolerance': 0.35, 'yaw_goal_tolerance': 6.28})
    ctrl['FollowPath'] = {
        'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        'desired_linear_vel': CRUISE,
        'lookahead_dist': 0.6, 'min_lookahead_dist': 0.35, 'max_lookahead_dist': 0.9,
        'lookahead_time': 1.5, 'use_velocity_scaled_lookahead_dist': False,
        'rotate_to_heading_angular_vel': 0.8, 'use_rotate_to_heading': True,
        'rotate_to_heading_min_angle': 0.7, 'max_angular_accel': 2.0,
        'allow_reversing': False, 'transform_tolerance': 0.3,
        'min_approach_linear_velocity': 0.08, 'approach_velocity_scaling_dist': 0.5,
        'use_regulated_linear_velocity_scaling': True,
        'regulated_linear_scaling_min_radius': 0.9, 'regulated_linear_scaling_min_speed': 0.12,
        'use_cost_regulated_linear_velocity_scaling': True,
        'cost_scaling_dist': 0.5, 'cost_scaling_gain': 1.0, 'inflation_cost_scaling_factor': 3.5,
        'use_collision_detection': True, 'max_allowed_time_to_collision_up_to_carrot': 1.0,
    }

    inflation = {'plugin': 'nav2_costmap_2d::InflationLayer',
                 'cost_scaling_factor': 3.5, 'inflation_radius': INFLATION}
    static = {'plugin': 'nav2_costmap_2d::StaticLayer', 'map_topic': nav_map,
              'map_subscribe_transient_local': True, 'trinary_costmap': False,
              'subscribe_to_updates': False}

    def scan_layer(rng):
        # The 2D scan only sees 0.25-1.5 m above the floor: walls, rubble, other robots.
        return {'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
                'observation_sources': 'scan',
                'scan': {'topic': f'/{ns}/scan', 'data_type': 'LaserScan', 'marking': True,
                         'clearing': True, 'max_obstacle_height': 2.0,
                         'raytrace_max_range': rng + 0.5, 'obstacle_max_range': rng,
                         'obstacle_min_range': 0.45, 'raytrace_min_range': 0.0}}

    lc = d['local_costmap']['local_costmap']['ros__parameters']
    for k in ('voxel_layer', 'static_layer'):
        lc.pop(k, None)
    lc.update({'global_frame': odom_frame, 'robot_base_frame': base, 'robot_radius': ROBOT_RADIUS,
               'width': 4, 'height': 4, 'update_frequency': 5.0,
               'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
               'static_layer': static, 'obstacle_layer': scan_layer(3.0),
               'inflation_layer': inflation})

    gc = d['global_costmap']['global_costmap']['ros__parameters']
    gc.update({'global_frame': map_frame, 'robot_base_frame': base, 'robot_radius': ROBOT_RADIUS,
               'track_unknown_space': True, 'update_frequency': 1.0,
               'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
               'static_layer': static, 'obstacle_layer': scan_layer(5.0),
               'inflation_layer': inflation})

    planner = d['planner_server']['ros__parameters']
    planner['GridBased'].update({'tolerance': 0.5, 'allow_unknown': True, 'use_astar': True})

    beh = d['behavior_server']['ros__parameters']
    beh.update({'global_frame': odom_frame, 'robot_base_frame': base,
                'max_rotational_vel': 0.8, 'min_rotational_vel': 0.3, 'rotational_acc_lim': 2.0})

    keep = ['bt_navigator', 'bt_navigator_navigate_through_poses_rclcpp_node',
            'bt_navigator_navigate_to_pose_rclcpp_node', 'controller_server',
            'local_costmap', 'global_costmap', 'planner_server', 'behavior_server']
    params = {k: copy.deepcopy(d[k]) for k in keep}
    for v in params.values():
        _set_sim_time(v)
    return {ns: params}


def _set_sim_time(d):
    for k, v in d.items():
        if k == 'ros__parameters':
            v['use_sim_time'] = True
        elif isinstance(v, dict):
            _set_sim_time(v)


def write(ns, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    paths = {}
    for name, fn in (('slam', slam_params), ('nav2', nav2_params)):
        paths[name] = os.path.join(out_dir, f'{ns}_{name}.yaml')
        with open(paths[name], 'w') as f:
            yaml.safe_dump(fn(ns), f, sort_keys=False)
    return paths
