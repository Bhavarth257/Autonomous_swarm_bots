# Rescue swarm: autonomous victim search in a collapsed building

ROS 2 Humble + Gazebo Fortress. Three RHex-style hexabots explore a procedurally
generated collapsed building, climbing over low debris, and locate victims: visible
ones with a camera + YOLO, buried ones with a thermal camera. A mission dashboard
starts the autonomous search with one button.

## Status

| Step | What | State |
|---|---|---|
| 1 | World generator, multi-robot bringup | done |
| 1b | Hexabot (6 C-legs, tripod gait, climbing), climbable debris in worlds | done |
| 2 | SLAM + traversability map + Nav2 + frontier exploration (1 robot) | next |
| 3 | Perception: YOLOv8 (RGB) + thermal, fused victim candidates | |
| 4 | Decentralized 3-robot swarm, range-limited comms | |
| 5 | Mission dashboard (Start/Stop/Return, live status, victims, coverage) | |
| 6 | Scoring against ground truth, batch runs | |

## The hexabot

RHex-style: a 0.54 m body on six rotating C-legs ("whegs"). The `HexGait` Gazebo plugin
([src/rescue_hexabot/src/HexGait.cc](src/rescue_hexabot/src/HexGait.cc)) turns `cmd_vel`
into an alternating-tripod gait; each leg's foot is an arc centred on the hip, so the
body rolls level during stance, and the open side of the C hooks over edges.

Measured on a test course (Gazebo, 4 ms physics step):

| | |
|---|---|
| cruise | 0.70 m/s at 0.7 commanded, body bob 7 mm |
| turn in place | 1.57 rad/s at 1.5 commanded |
| climbs | 6/10/13 cm blocks, 13 cm beams, gravel, 22 cm mounds via ramps |
| 20 cm block | gets on top slowly (max tilt 41 deg), stays upright |

While climbing, the gait slows with body tilt (down to 35 % at 30 deg) and stops
driving past 40 deg nose-up, so the robot settles back instead of flipping.

Sensors: 16-ring 3D lidar (-25..+15 deg, 10 Hz), RGB camera (640x480, 10 Hz), IMU.
`hexabot_odom` turns the simulator pose into realistic legged odometry: roll, pitch and
height accurate, x/y/yaw drifting (~1-2 % of distance travelled).

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

# drive r2 (the one lined up with the entrance) by keyboard, at cruise speed
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r cmd_vel:=/r2/cmd_vel -p speed:=0.7 -p turn:=1.5
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

The same seed always gives the same building. Obstacles are either *climbable* (low
blocks, fallen joists, gravel, ramped rubble mounds; tan in the preview) or *blocking*
(walls, tall rubble, leaning slabs, beams, victims; dark). Every layout is checked so
that all rooms and victims can be reached from the entrance, climbing where needed.

## Robot topics (per robot `rN`)

| Topic | |
|---|---|
| `/rN/cmd_vel` | drive command |
| `/rN/odom` | legged odometry |
| `/rN/lidar/points` | 3D lidar cloud |
| `/rN/scan` | 2D scan cut from the cloud 0.25-1.5 m above the ground (walls, tall rubble) |
| `/rN/camera/image_raw`, `/rN/camera/camera_info`, `/rN/imu`, `/rN/joint_states` | |
| `/rN/ground_truth/odom` | simulator pose, for scoring only |

TF frames are prefixed (`rN/base_link`) on the shared `/tf`:
`world -> rN/map` (known start pose) `-> rN/odom -> rN/base_footprint` (level, at ground)
`-> rN/base_link` (body roll/pitch).

## Tests

```bash
cd src/rescue_world && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test
```

(`PYTEST_DISABLE_PLUGIN_AUTOLOAD` works around the user-installed pytest 9 clashing with
ROS's launch_testing plugin.)
