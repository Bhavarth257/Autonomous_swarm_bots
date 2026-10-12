"""Collapsed-building world + N rescue robots in Gazebo Fortress.

  ros2 launch rescue_bringup sim.launch.py seed:=4 num_robots:=3 gui:=true rviz:=true

The world is generated from `seed` (same seed -> same building and victims) into
~/.ros/rescue_worlds/. Each robot rN gets:
  /rN/cmd_vel /rN/odom /rN/scan /rN/imu /rN/camera/{image_raw,camera_info} /rN/joint_states
and TF frames rN/odom -> rN/base_footprint -> ... on the shared /tf.

The robots start at known poses outside the entrance, so `world -> rN/map` is a
static transform; until SLAM runs (`slam:=false`), `rN/map -> rN/odom` is identity.
"""
import json
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rescue_bringup import rviz_config
from rescue_world.generate import DEFAULT_OUT, generate


def _robot(ns, spawn, description, slam):
    sim_time = {'use_sim_time': True}
    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=ns,
             parameters=[{'robot_description': description, 'frame_prefix': f'{ns}/', **sim_time}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', ns, '-topic', f'/{ns}/robot_description',
                        '-x', str(spawn['x']), '-y', str(spawn['y']), '-z', '0.01',
                        '-Y', str(spawn['yaw'])]),
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


def _bridge_topics(ns):
    return [
        f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        f'/{ns}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        f'/{ns}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        f'/{ns}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        f'/{ns}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        f'/{ns}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        f'/{ns}/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        f'/{ns}/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
    ]


def _setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    seed, n = int(arg('seed')), int(arg('num_robots'))
    gui, rviz, slam = (arg(k) == 'true' for k in ('gui', 'rviz', 'slam'))

    paths = generate(seed=seed, out_dir=DEFAULT_OUT, n_robots=max(n, 3))
    with open(paths['json']) as f:
        gt = json.load(f)
    print(f'[rescue] world {paths["sdf"]} | preview {paths["preview"]}')

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {"" if gui else "-s "}{paths["sdf"]}'}.items())

    xacro_file = os.path.join(get_package_share_directory('rescue_description'),
                              'urdf', 'rescue_bot.urdf.xacro')
    namespaces = [f'r{k + 1}' for k in range(n)]
    actions = [gz]
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    remaps = []
    for k, ns in enumerate(namespaces):
        description = xacro.process_file(xacro_file, mappings={'ns': ns}).toxml()
        actions += _robot(ns, gt['spawns'][k], description, slam)
        bridge_args += _bridge_topics(ns)
        remaps.append((f'/{ns}/tf', '/tf'))

    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
                        arguments=bridge_args, remappings=remaps,
                        parameters=[{'use_sim_time': True}], output='screen'))

    if rviz:
        cfg = os.path.join(DEFAULT_OUT, f'rviz_{n}.rviz')
        with open(cfg, 'w') as f:
            f.write(rviz_config.build(namespaces))
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', cfg],
                            parameters=[{'use_sim_time': True}]))
    return actions


NVIDIA_EGL = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'


def generate_launch_description():
    # Hybrid laptops: make headless rendering go straight to the NVIDIA EGL driver
    # instead of trying Mesa first ("failed to create dri2 screen").
    gpu_env = [SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES', NVIDIA_EGL)] \
        if os.path.exists(NVIDIA_EGL) else []
    return LaunchDescription(gpu_env + [
        DeclareLaunchArgument('seed', default_value='0', description='world seed'),
        DeclareLaunchArgument('num_robots', default_value='1'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='true once SLAM provides map -> odom'),
        OpaqueFunction(function=_setup),
    ])
