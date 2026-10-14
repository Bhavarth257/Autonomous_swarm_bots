"""Collapsed-building world + N rescue hexapods in Gazebo Fortress.

  ros2 launch rescue_bringup sim.launch.py seed:=4 num_robots:=3 gui:=true rviz:=true

The world is generated from `seed` (same seed -> same building and victims) into
~/.ros/rescue_worlds/. Each hexapod rN gets:
  /rN/cmd_vel                 walk command: linear.x, linear.y (crab walk), angular.z
  /rN/odom                    drifting legged odometry (hexabot_odom)
  /rN/lidar/points            16-ring 3D lidar
  /rN/scan                    2D scan cut from the cloud 0.25-1.5 m above the ground
                              (walls and tall rubble only; climbable debris is below it)
  /rN/imu /rN/camera/{image_raw,camera_info} /rN/thermal/{image_raw,camera_info}
  /rN/joint_states
  /rN/ground_truth/odom       simulator pose, for scoring only
and TF frames rN/odom -> rN/base_footprint -> rN/base_link -> ... on the shared /tf.

The robots start at known poses outside the entrance, so `world -> rN/map` is a
static transform; until SLAM runs (`slam:=false`), `rN/map -> rN/odom` is identity.
"""
import json
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
                            UnsetEnvironmentVariable)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rescue_bringup import rviz_config
from rescue_navigation.robot_profiles import DEFAULT, profile
from rescue_world.generate import DEFAULT_OUT, generate_from_launch


def _robot(ns, spawn, description, slam, odom_drift, prof):
    sim_time = {'use_sim_time': True}
    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=ns,
             parameters=[{'robot_description': description, 'frame_prefix': f'{ns}/', **sim_time}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', ns, '-topic', f'/{ns}/robot_description',
                        '-x', str(spawn['x']), '-y', str(spawn['y']), '-z', str(prof['spawn_z']),
                        '-Y', str(spawn['yaw'])]),
        Node(package='rescue_hexabot', executable='hexabot_odom', namespace=ns,
             parameters=[{'body_height': prof['body_height'], 'drift': odom_drift, **sim_time}]),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='cloud_to_scan', namespace=ns,
             remappings=[('cloud_in', 'lidar/points'), ('scan', 'scan')],
             # The SLAM scan starts above a hexapod's height (0.31 m): a teammate walking past
             # would otherwise become a wall in the map, and one recorded closer than the
             # scan's 0.45 m minimum range is never cleared again, trapping the robot.
             # Anything lower is the 3D height map's job (traversability).
             parameters=[{'target_frame': f'{ns}/base_footprint', 'transform_tolerance': 0.05,
                          'min_height': 0.40, 'max_height': 1.5,
                          'angle_min': -3.14159, 'angle_max': 3.14159, 'angle_increment': 0.0087,
                          'scan_time': 0.1, 'range_min': 0.45, 'range_max': 15.0,
                          'use_inf': True, **sim_time}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name=f'{ns}_world_tf',
             arguments=['--x', str(spawn['x']), '--y', str(spawn['y']), '--yaw', str(spawn['yaw']),
                        '--frame-id', 'world', '--child-frame-id', f'{ns}/map'],
             parameters=[sim_time]),
    ]
    if not slam:
        actions.append(
            Node(package='tf2_ros', executable='static_transform_publisher', name=f'{ns}_map_odom_tf',
                 arguments=['--frame-id', f'{ns}/map', '--child-frame-id', f'{ns}/odom'],
                 parameters=[sim_time]))
    return actions


def _bridge_topics(ns, prof):
    stance = [f'/{ns}/stance@std_msgs/msg/Float64]ignition.msgs.Double'] if prof.get('stance_topic') else []
    return stance + [
        f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        f'/{ns}/terrain_ahead@std_msgs/msg/Float32]ignition.msgs.Float',
        f'/{ns}/ground_truth/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        f'/{ns}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        f'/{ns}/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        f'/{ns}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        f'/{ns}/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        f'/{ns}/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        f'/{ns}/thermal/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        f'/{ns}/thermal/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
    ]


def _setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    seed, n = int(arg('seed')), int(arg('num_robots'))
    gui, rviz, slam, drift = (arg(k) == 'true' for k in ('gui', 'rviz', 'slam', 'odom_drift'))

    paths = generate_from_launch(seed, n, int(arg('grid')), arg('difficulty'), arg('victims_file'))
    with open(paths['json']) as f:
        gt = json.load(f)
    print(f'[rescue] world {paths["sdf"]} | preview {paths["preview"]}')

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {"" if gui else "-s "}{paths["sdf"]}'}.items())

    # Gazebo's server can outlive a launch that shuts itself down (e.g. after an
    # evaluation); make sure this world's server goes with it.
    stop_gz = RegisterEventHandler(OnShutdown(on_shutdown=[ExecuteProcess(
        cmd=['pkill', '-INT', '-f', f'gazebo -r .*{paths["sdf"]}'], output='log')]))

    prof = profile(arg('robot'))
    xacro_file = os.path.join(get_package_share_directory('rescue_description'), 'urdf', prof['xacro'])
    namespaces = [f'r{k + 1}' for k in range(n)]
    actions = [gz, stop_gz]
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    for k, ns in enumerate(namespaces):
        description = xacro.process_file(xacro_file, mappings={'ns': ns}).toxml()
        actions += _robot(ns, gt['spawns'][k], description, slam, drift, prof)
        bridge_args += _bridge_topics(ns, prof)

    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
                        arguments=bridge_args, parameters=[{'use_sim_time': True}], output='screen'))

    if rviz:
        cfg = os.path.join(DEFAULT_OUT, f'rviz_{n}.rviz')
        with open(cfg, 'w') as f:
            f.write(rviz_config.build(namespaces))
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', cfg],
                            parameters=[{'use_sim_time': True}]))
    return actions


NVIDIA_EGL = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
SNAP_GTK_VARS = ['GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'GIO_MODULE_DIR',
                 'GDK_PIXBUF_MODULE_FILE', 'GDK_PIXBUF_MODULEDIR', 'GSETTINGS_SCHEMA_DIR', 'LOCPATH']


def _clean_env():
    actions = []
    # Hybrid laptops: make headless rendering go straight to the NVIDIA EGL driver
    # instead of trying Mesa first ("failed to create dri2 screen").
    if os.path.exists(NVIDIA_EGL):
        actions.append(SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES', NVIDIA_EGL))
    # Gazebo transport discovers peers by multicast on the default network interface;
    # if Wi-Fi drops mid-run the bridge loses Gazebo and the clock freezes. Everything
    # runs on this machine, so keep it on loopback (unless the user chose otherwise).
    if 'IGN_IP' not in os.environ:
        actions.append(SetEnvironmentVariable('IGN_IP', '127.0.0.1'))
    # FastDDS without shared memory (see config/fastdds_udp.xml), unless the user set a profile.
    if 'FASTRTPS_DEFAULT_PROFILES_FILE' not in os.environ:
        actions.append(SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', os.path.join(
            get_package_share_directory('rescue_bringup'), 'config', 'fastdds_udp.xml')))
    # A terminal inside snap-packaged VS Code leaks the snap's GTK paths, and RViz then
    # loads the snap's libpthread and dies ("undefined symbol: __libc_pthread_init").
    if os.environ.get('SNAP_NAME') == 'code':
        actions += [UnsetEnvironmentVariable(v) for v in SNAP_GTK_VARS if v in os.environ]
        for v in ('XDG_DATA_DIRS', 'XDG_CONFIG_DIRS'):
            orig = os.environ.get(f'{v}_VSCODE_SNAP_ORIG')
            if orig is not None:
                actions.append(SetEnvironmentVariable(v, orig))
    return actions


def generate_launch_description():
    return LaunchDescription(_clean_env() + [
        DeclareLaunchArgument('seed', default_value='0', description='world seed'),
        DeclareLaunchArgument('grid', default_value='4', description='rooms per side (4 -> 4x4)'),
        DeclareLaunchArgument('difficulty', default_value='normal', description='normal | hard'),
        DeclareLaunchArgument('victims_file', default_value='',
                              description='JSON list of {x, y, posture, buried, yaw}: your own victims'),
        DeclareLaunchArgument('num_robots', default_value='1'),
        DeclareLaunchArgument('robot', default_value=DEFAULT, description='rover | hexapod | wheelleg'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='true once SLAM provides map -> odom'),
        DeclareLaunchArgument('odom_drift', default_value='true',
                              description='false: odometry equals ground truth (debugging)'),
        OpaqueFunction(function=_setup),
    ])
