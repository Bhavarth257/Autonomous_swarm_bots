"""Per-robot SLAM Toolbox and Nav2 parameters for a namespaced wheel-leg robot.

Starts from the installed defaults and overrides frames, topics, the costmap layers
(fused nav_map + live 2D scan) and the controller tuning.
Everything is nested under the robot namespace, so one file serves /rN/<node>.
"""
import copy
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from rescue_navigation.robot_config import ROBOT



# Nav2's default tree, with quick recoveries: its 1.57 rad spin fails in a doorway (the
# body's corners touch the jambs), and backing up 0.3 m at 0.05 m/s plus a 5 s wait stood a robot
# still for 10-15 s at every narrow door. Three retries, not six: an unreachable frontier
# should fail fast; the explorer then blacklists it and picks another.
BT_XML = """<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="3" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <BackUp backup_dist="0.20" backup_speed="0.15"/>
          <Wait wait_duration="1"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
"""


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


def nav2_params(ns, bt_xml=None):
    prof = ROBOT
    d = _load('nav2_bringup', 'params', 'nav2_params.yaml')
    map_frame, odom_frame, base = f'{ns}/map', f'{ns}/odom', f'{ns}/base_footprint'
    nav_map = f'/{ns}/nav_map'

    bt = d['bt_navigator']['ros__parameters']
    bt.update({'global_frame': map_frame, 'robot_base_frame': base, 'odom_topic': f'/{ns}/odom'})
    if bt_xml:
        bt['default_nav_to_pose_bt_xml'] = bt_xml

    ctrl = d['controller_server']['ros__parameters']
    ctrl['controller_frequency'] = 20.0
    # A late map->odom transform (SLAM behind under load) used to abort the path at once
    # ("Controller patience exceeded") and the explorer then backed away as if stuck. Ride
    # out a second of that instead.
    ctrl['failure_tolerance'] = 1.0
    ctrl['progress_checker'].update({'required_movement_radius': 0.3, 'movement_time_allowance': 20.0})
    ctrl['general_goal_checker'].update({'xy_goal_tolerance': 0.35, 'yaw_goal_tolerance': 6.28})
    ctrl['FollowPath'] = {
        'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        'desired_linear_vel': prof['cruise'],
        # Short look-ahead in doorways: a long one cuts the corner, the collision check
        # then sees the jamb ahead and stops the robot.
        'lookahead_dist': 0.6, 'min_lookahead_dist': 0.4, 'max_lookahead_dist': 0.9,
        'lookahead_time': 1.5, 'use_velocity_scaled_lookahead_dist': True,
        'rotate_to_heading_angular_vel': prof['rotate_vel'], 'use_rotate_to_heading': True,
        'rotate_to_heading_min_angle': 1.0, 'max_angular_accel': 3.0,
        'allow_reversing': False, 'transform_tolerance': 0.5,
        'min_approach_linear_velocity': 0.08, 'approach_velocity_scaling_dist': 0.5,
        'use_regulated_linear_velocity_scaling': True,
        'regulated_linear_scaling_min_radius': 0.9, 'regulated_linear_scaling_min_speed': prof['min_speed'],
        'use_cost_regulated_linear_velocity_scaling': True,
        'cost_scaling_dist': 0.5, 'cost_scaling_gain': 1.0, 'inflation_cost_scaling_factor': prof['cost_scaling'],
        # With a rectangular footprint this check stops the robot whenever its outline
        # touches the inflated band around a wall, i.e. in every doorway and tight room (~40 %
        # of control cycles). The planner only keeps the centre clear, so the inflation keeps
        # paths in the middle (robot_config cost_scaling) and wall_guard handles contact.
        'use_collision_detection': False, 'max_allowed_time_to_collision_up_to_carrot': 1.0,
    }

    inflation = {'plugin': 'nav2_costmap_2d::InflationLayer',
                 'cost_scaling_factor': prof['cost_scaling'], 'inflation_radius': prof['inflation']}
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
    body = {'footprint': str(prof['footprint'])}
    lc.update({'global_frame': odom_frame, 'robot_base_frame': base, **body,
               'width': 4, 'height': 4, 'update_frequency': 5.0,
               'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
               'static_layer': static, 'obstacle_layer': scan_layer(3.0),
               'inflation_layer': inflation})

    gc = d['global_costmap']['global_costmap']['ros__parameters']
    global_plugins = (['static_layer', 'obstacle_layer', 'inflation_layer'] if prof['global_scan']
                      else ['static_layer', 'inflation_layer'])
    gc.update({'global_frame': map_frame, 'robot_base_frame': base, **body,
               'track_unknown_space': True, 'update_frequency': 1.0,
               'plugins': global_plugins,
               'static_layer': static, 'obstacle_layer': scan_layer(5.0),
               'inflation_layer': inflation})

    planner = d['planner_server']['ros__parameters']
    planner['GridBased'].update({'tolerance': 0.5, 'allow_unknown': True, 'use_astar': True})

    beh = d['behavior_server']['ros__parameters']
    beh.update({'global_frame': odom_frame, 'robot_base_frame': base,
                'max_rotational_vel': prof['rotate_vel'], 'min_rotational_vel': 0.4, 'rotational_acc_lim': 3.0})

    # Smooth the controller's output (acceleration / deceleration limits) so the drive never
    # sees a stop-go command stream.
    d['velocity_smoother']['ros__parameters'].update({
        'smoothing_frequency': 20.0, 'scale_velocities': False, 'feedback': 'OPEN_LOOP',
        'max_velocity': prof['max_vel'], 'min_velocity': [-0.3, -prof['max_vel'][1], -prof['max_vel'][2]],
        'max_accel': [0.5, 0.5, 3.0], 'max_decel': [-0.8, -0.8, -4.0],
        'odom_topic': f'/{ns}/odom', 'odom_duration': 0.1, 'deadband_velocity': [0.0, 0.0, 0.0],
        'velocity_timeout': 1.0})

    keep = ['bt_navigator', 'bt_navigator_navigate_through_poses_rclcpp_node', 'velocity_smoother',
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
    paths = {'bt': os.path.join(out_dir, 'navigate_to_pose_quick_recovery.xml')}
    with open(paths['bt'], 'w') as f:
        f.write(BT_XML)
    for name, data in (('slam', slam_params(ns)), ('nav2', nav2_params(ns, paths['bt']))):
        paths[name] = os.path.join(out_dir, f'{ns}_{name}.yaml')
        with open(paths[name], 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
    return paths
