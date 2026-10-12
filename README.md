# Rescue swarm: autonomous victim search in a collapsed building

ROS 2 Humble + Gazebo Fortress. Three small robots explore a procedurally generated
collapsed building and locate victims: visible ones with a camera + YOLO, buried ones
with a thermal camera.

## Status

| Step | What | State |
|---|---|---|
| 1 | World generator, robot model, multi-robot bringup | done |
| 2 | SLAM Toolbox + Nav2 + frontier exploration (1 robot) | next |
| 3 | Perception: YOLOv8 (RGB) + thermal, fused victim candidates | |
| 4 | Decentralized 3-robot swarm, range-limited comms | |
| 5 | Scoring against ground truth, batch runs | |

## Build

```bash
cd ~/analog_project
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# Gazebo + RViz, one robot, world seed 0
ros2 launch rescue_bringup sim.launch.py

# three robots, another building
ros2 launch rescue_bringup sim.launch.py seed:=4 num_robots:=3

# drive r1 by keyboard (separate terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/r1/cmd_vel
```

Launch arguments: `seed` (world), `num_robots` (1-3), `gui` (Gazebo window), `rviz`, `slam`.

### Worlds only

```bash
ros2 run rescue_world generate_world --seed 4
```

Writes into `~/.ros/rescue_worlds/`:

- `rescue_<seed>.sdf`: the Gazebo world
- `rescue_<seed>.json`: ground truth (victims, decoys, spawns, explorable area)
- `rescue_<seed>_map.pgm/.yaml`: ground-truth occupancy map
- `rescue_<seed>_preview.png`: top-down view (red = visible victim, orange = buried, blue = hot decoy)

The same seed always gives the same building. Every layout is checked so that all rooms
and victims can be reached from the entrance, and buried victims can be observed.

## Robot topics (per robot `rN`)

`/rN/cmd_vel`, `/rN/odom`, `/rN/scan`, `/rN/imu`, `/rN/camera/image_raw`,
`/rN/camera/camera_info`, `/rN/joint_states`. TF frames are prefixed (`rN/base_link`) on the
shared `/tf`; `world -> rN/map` is the known start pose outside the entrance.

## Tests

```bash
cd src/rescue_world && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test
```

(`PYTEST_DISABLE_PLUGIN_AUTOLOAD` works around the user-installed pytest 9 clashing with
ROS's launch_testing plugin.)
