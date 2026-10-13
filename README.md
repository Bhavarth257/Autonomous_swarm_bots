# Rescue swarm: autonomous victim search in a collapsed building

ROS 2 Humble + Gazebo Fortress. Three 18-DOF spider hexapods enter a procedurally
generated collapsed building, map it, walk over low debris, find victims (visible
ones with a camera + YOLOv8, buried ones with a thermal camera), share what they know
over a range-limited radio without any central controller, and walk back out. A web
mission control (http://localhost:8080) starts the whole search with one button, shows
every robot's cameras and the combined map, and lets you take over any robot, drive it,
or drop new victims into the running world.

![hexapod](docs/hexapod.png)

![mission dashboard](docs/dashboard.png)

## Results

Measured on the earlier 3x3-room, normal-difficulty buildings (the default is now 4x4).
Three robots, 15 min limit, scored against the generator's ground truth by
`rescue_eval` (each building has 4 visible and 2 buried victims plus one hot decoy;
development was done mostly on seed 4, and the last perception change was guided by
seed 1's results):

| seed | victims found | false positives | floor explored | mission time | robots home |
|---|---|---|---|---|---|
| 1 | 5/6 (4/4 visible, 1/2 buried) | 1 | 100 % | 19.1 min* | 2/3 |
| 2 | 4/6 (2/4 visible, 2/2 buried) | 2 | 100 % | 10.5 min | 3/3 |
| 3 | 6/6 (4/4 visible, 2/2 buried) | 2 | 100 % | 7.2 min | 3/3 |
| 4 | 6/6 (4/4 visible, 2/2 buried) | 5 | 100 % | 17.0 min* | 2/3 |

\* hit the time limit while one robot was still walking home. Seed 4 ran before the
distance-weighted placement that halved false positives on seeds 1-3 (9 -> 5).
Mean victim position error 0.6-0.7 m; the hot decoy was reported as a hazard, never
as a victim, in every run. Simulation speed with everything running: ~0.85x real time
on a laptop with an i5-12500H and an RTX 3050 (YOLO ~15 ms per frame on the GPU).

**Hard 4x4 building** (`grid:=4 difficulty:=hard`, seed 4, 24 x 20 m, smoke, dark rooms,
0.85 m doors, stepped debris), three robots, 15 min limit, two consecutive runs:

| run | victims found | false positives | mean error | floor explored | robots home |
|---|---|---|---|---|---|
| 1 | 6/6 (4/4 visible, 2/2 buried) | 1 | 0.38 m | 100 % | 1/3 |
| 2 | 6/6 (4/4 visible, 2/2 buried) | 1 | 0.31 m | 100 % | 1/3 |

The hot decoy was a hazard, never a victim. Robots that were still deep inside the
building at the time limit didn't make it out; the early, distance-based return and the
"own trail stays walkable" rule were added after these runs and still need a full
scored run to confirm.

Known limitations:

- Getting all three robots back out of a large rubble-filled building in time (see above).
- A heavily loaded or very hot laptop slows the simulation until odometry arrives too
  late to use and a robot never starts; let it cool down between long runs.
- Far-away sightings are still placed less accurately than close ones (the camera sits
  20 cm above the floor, so a pixel of error is a lot of floor).
- A victim seen only from a distance can be missed, and a buried victim with a limb
  showing may be reported as visible (and the reverse).
- Results vary from run to run (physics, timing); a robot wedged against rubble
  sometimes doesn't make it home before the time limit.


## Quick start

```bash
# once: Rescue Randy victim models and YOLO (see "Setup" for details)
ign fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rescue Randy"
ign fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rescue Randy Sitting"
pip install --user ultralytics "numpy<2"

cd ~/analog_project
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch rescue_bringup mission.launch.py seed:=4 num_robots:=3
```

Gazebo, RViz and **mission control** (http://localhost:8080 in your browser) open.
Press **Start all**: the three robots explore, report victims and come back by
themselves. See [Mission control](#mission-control) for everything else.

Harder building, and your own victims:

```bash
ros2 launch rescue_bringup mission.launch.py seed:=4 num_robots:=3 difficulty:=hard
ros2 launch rescue_bringup mission.launch.py seed:=4 num_robots:=3 \
    victims_file:=$HOME/rescue_reports/scenario_20261013_120000.json
```

Useful launch arguments:

| argument | default | |
|---|---|---|
| `seed` | 0 | building layout (same seed, same building and victims) |
| `grid` | 4 | rooms per side (4 = 4x4 rooms, 24 x 20 m) |
| `difficulty` | normal | `hard`: narrow doors, stepped debris, smoke, dark rooms |
| `victims_file` | | JSON with your own victims instead of the random ones |
| `num_robots` | 1 | 1-3 hexapods |
| `auto_start` | false | start exploring without pressing START |
| `time_limit` | 1200 | seconds; each robot heads home early enough (from its distance to the entrance) to be out by then |
| `gui`, `rviz`, `dashboard` | true | Gazebo, RViz, web mission control |
| `port`, `open_browser` | 8080, true | mission control address; open a browser tab at start |
| `perception`, `swarm` | true | victim detector; radio + swarm agents |
| `radio_range`, `radio_wall_range` | 12, 4 | m: line of sight, through walls |
| `evaluate` | false | score against ground truth, write a report, shut down |

## Mission control

A web app served by the `dashboard` node (aiohttp + rclpy) at http://localhost:8080.
The robots are autonomous; everything below is optional.

- **Top bar**: mission time, building explored, victims found, robots active;
  **Start all / Stop all / Return all**; **Save report** (`~/rescue_reports/mission_<time>.md`
  with victim snapshots); **Save scenario** (see below).
- **Map**: all robots' maps combined in the building frame, drag to pan, scroll to zoom,
  **Fit**. Layers: map, trails, radio range, goals, victims. Smoke and dark rooms are
  hatched. Click a victim for its first-sighting snapshot, confidence and who found it;
  click a robot to select it.
- **Robot cards**: state, position, speed, distance walked, area explored, radio
  contact, current activity; start / stop / home / **Take over** per robot.
- **Cameras**: every robot's RGB (with YOLO boxes) and thermal (false colour); click to
  enlarge.
- **Victims** list with snapshots, **charts** (coverage and victims over time, distance
  walked per robot) and a **timeline** of events.

**Intervening.** **Take over** puts a robot in manual mode: drive it with the on-screen
joystick or the keyboard (**W/S** forward/back, **A/D** turn, **Q/E** crab walk; the
robot stops if commands stop for 0.4 s). Or pick **Send robot here** and click the map:
the robot walks there with Nav2 and holds. **Resume autonomy** hands it back to its
explorer, which carries on where it left off.

**Placing victims.** Pick **Place victim** (lying, sitting or buried) and click the map:
the victim appears in the running Gazebo world and in the ground truth used for scoring.
**Save scenario** writes all current victims to `~/rescue_reports/scenario_<time>.json`;
start a later run with `victims_file:=<that file>` (same `seed`) to test the same setup
again. You can also write the file yourself, in building coordinates (metres, origin
at the bottom-left outer corner, the entrance on the bottom wall):

```json
{"victims": [
  {"x": 3.0, "y": 2.5, "posture": "lying", "yaw": 30},
  {"x": 15.0, "y": 12.0, "posture": "sitting", "yaw": -90},
  {"x": 20.0, "y": 7.0, "buried": true}
]}
```

User-placed victims replace the random ones and may be anywhere, including places
the robots cannot reach.

The page also works from another computer on your network (http://<this-pc>:8080),
since the server listens on all interfaces; anyone who can open it can drive the robots.

## How it works

```
                 Gazebo Fortress (physics, 3D lidar, RGB + thermal cameras)
                                       │
   per robot rN ───────────────────────┼───────────────────────────────────────────
   HexWalk plugin (18-joint IK gait) ◄─ cmd_vel ── Nav2 (NavFn A*, Regulated Pure Pursuit)
   hexabot_odom (legged odometry)                      ▲ nav_map
   SLAM Toolbox (2D) ── map ──► traversability ────────┘ (height map + SLAM + geofence)
   explorer (frontiers, own + team) ◄── team/state ── swarm_agent ◄─► radio channel ◄─► other robots
   ─────────────────────────────────────────────────────────────────────────────────
   shared: victim_detector (YOLOv8 on GPU + thermal + lidar placement, per-robot outputs)
           radio (who can hear whom: range + line of sight)   dashboard   evaluator
```

| Package | What it does |
|---|---|
| `rescue_world` | Seeded building generator: 4x4 rooms (`grid`), doors (some buried in rubble), breaches, climbable debris (blocks, joists, gravel, ramped mounds) and blocking debris, Rescue Randy victims lying or sitting (some buried under warm debris), a hot decoy; `hard` adds 0.85 m doors, stepped debris, smoke (blinds the RGB camera, not thermal or lidar) and dark rooms under a collapsed roof; or your own victims from a file. Writes the SDF world and the ground truth. |
| `rescue_description` | Hexapod URDF: kit-style body, 6 legs x 3 joints, 16-ring 3D lidar, RGB + thermal cameras, IMU. |
| `rescue_hexabot` | `HexWalk` Gazebo plugin (tripod gait, IK, body levelling, climb slow-down, 30 Hz joint states) and `hexabot_odom` (drifting legged odometry). |
| `rescue_navigation` | `traversability` (height map -> nav_map), `explorer` (mission state machine, frontiers, unstick, Nav2 self-healing, operator take-over and go-to), SLAM/Nav2 configuration. |
| `rescue_perception` | `victim_detector`: YOLOv8 persons + thermal body blobs, placed with lidar and the floor plane, fused into victims per robot. |
| `rescue_swarm` | `radio` (simulated channel) and `swarm_agent` (gossip: poses, claimed goals, explored areas, victims). |
| `rescue_dashboard` | Web mission control (aiohttp server + ROS 2 node, page in `web/`). |
| `rescue_eval` | `evaluate` (score against ground truth) and `run_batch` (many seeds headless). |

### Hexapod

An 18-DOF spider hexapod styled after hobby kits. All joints are physically simulated:
the `HexWalk` plugin computes inverse kinematics for a tripod gait from `cmd_vel`
(forward, sideways, turning); feet lift 11 cm straight up, swing, and drop, so they
clear edges. Each foot comes down until it actually touches something, so a leg landing
on a block simply stands higher instead of stubbing; the body rises with the ground
under its feet, the gait slows down ahead of an obstacle the lidar sees
(`terrain_ahead`), and the IMU levels the body over debris. Nav2's velocity smoother
ramps speed changes, so the walk does not jerk between stops and starts.
Measured: 0.40 m/s cruise, 0.92 rad/s turning, climbs 12 cm blocks, 9 cm joists, gravel
and 20 cm mounds over 15 deg ramps without flipping.

### Mapping and navigation (per robot)

- **SLAM Toolbox** builds the wall map from a 2D scan cut out of the 3D cloud at
  0.4-1.5 m height (above the robots, so teammates never end up in the map as walls), and corrects odometry drift.
- **traversability** keeps a 10 cm height map from the 3D lidar and publishes `nav_map`:
  lethal for walls, anything taller than 40 cm and the top of any step taller than
  14 cm (victims, mound edges); 20-70 for climbable steps so paths prefer flat floor;
  "probably free" where SLAM saw open space but the floor isn't seen yet; lethal
  outside the building except near the start (geofence).
- **Nav2**: NavFn A* (may plan through unknown space), Regulated Pure Pursuit at
  0.35 m/s, spin/back-up recoveries.
- **explorer**: frontiers of its own map it can actually reach (obstacles grown by the
  robot radius); when those are used up, the edge of what *nobody in the team* has
  seen; blacklists unreachable spots for 90 s; crab-walks away from obstacles when
  repeatedly stuck; restarts Nav2 if its bringup failed; walks home when nothing is left.

### Victim detection

- **YOLOv8n** (COCO "person") on the RGB camera, on the GPU, one model for the team.
- **Thermal camera** (Gazebo, 16-bit centi-kelvin): blobs at 28-45 C are body heat
  (or debris warmed by a buried body); above 52 C is a hazard (fire), never a victim.
- **Placement**: the bottom of the detection box is where the body touches the floor;
  the ray through it meets the floor plane, and lidar points at that distance refine
  the position (a lying body is often below the lidar rings, so the wall behind it
  must not be taken for it).
- **Fusion** per robot: visible = YOLO saw a person; buried = body heat seen repeatedly
  from 2+ places with no person visible; ghost tracks next to a stronger one are folded
  in; confidence grows with sightings and viewpoints.

### Swarm

No robot is in charge. The **radio** node is the physics of the channel: robots hear
each other within 12 m with line of sight, or 4 m through walls. Each **swarm_agent**
broadcasts its team view once a second and merges what it hears (newest entry wins),
so news travels through teammates: positions, claimed goals, a 0.5 m explored grid per
robot, and victims. Explorers skip areas teammates covered and keep away from their
positions and goals.

### Evaluation

```bash
ros2 run rescue_eval run_batch 1 2 3 4 5        # ROBOTS=3 TIME_LIMIT=900 by default
```

Runs each seed headless with `evaluate:=true` and writes `~/rescue_reports/seed<N>_<R>robots.md`
plus `summary_<R>robots.md`: victims found per type, false positives, position error,
hazard handling, floor coverage, mission time, robots back at the start.

## Setup details

- **ROS 2 Humble, Gazebo Fortress** with `ros-humble-ros-gz`, `slam-toolbox`, `navigation2`,
  `nav2-bringup`, `pointcloud-to-laserscan`, `xacro` (all apt packages).
- **Victim models**: the two `ign fuel download` commands above put Rescue Randy in
  `~/.ignition/fuel`. Without them victims are built from simple shapes (thermal
  detection still works; YOLO will rarely recognise them).
- **YOLO**: `pip install --user ultralytics "numpy<2"` (keep NumPy 1.x, ROS Humble's
  Python packages are built against it). The weights (`yolov8n.pt`) download to
  `~/.ros/rescue_models/` on first start. Without ultralytics the detector runs
  thermal-only.

## Troubleshooting

- **RViz crashes from the VS Code terminal** ("undefined symbol __libc_pthread_init"):
  snap-packaged VS Code leaks GTK paths; the launch files remove them.
- **"No ROS2 environment active"** in VS Code's status bar comes from the ros2env
  extension (Docker environments); it doesn't affect anything here.
- **Everything freezes when Wi-Fi drops**: Gazebo transport is pinned to loopback
  (`IGN_IP=127.0.0.1`) by the launch file for exactly this reason.
- **Simulation much slower than usual, log says `CUDA initialization: CUDA unknown error`**:
  the NVIDIA driver lost its unified-memory module (typically after suspend), so YOLO
  runs on the CPU. `sudo rmmod nvidia_uvm && sudo modprobe nvidia_uvm` (or a reboot)
  brings the GPU back; `python3 -c "import torch; print(torch.cuda.is_available())"`
  should print `True`.
- **Robots never start moving, and the log repeats `no rN/map -> rN/odom transform yet` or
  `Message Filter dropping message ... queue is full`**: a run that crashed or was killed
  left stale shared-memory files behind, and ROS messages get lost. With nothing ROS
  running, `fastdds shm clean`, then launch again.
- **Mission control shows "address already in use"**: another run is still up, or
  something else uses 8080; pass `port:=8090`.
- **No cameras on the page**: they appear once Gazebo renders its first frames
  (10-20 s after start).
- **Stop with Ctrl+C.**

## Tests

```bash
source /opt/ros/humble/setup.bash
for p in rescue_world rescue_navigation rescue_perception rescue_swarm rescue_eval; do
  (cd src/$p && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
     PYTHONPATH=$PWD:../rescue_perception:../rescue_navigation:$PYTHONPATH python3 -m pytest -q test)
done
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD` works around a user-installed pytest 9 clashing with
ROS's launch_testing plugin.

## Robot topics (per robot `rN`)

| Topic | |
|---|---|
| `/rN/cmd_vel` | walk command: `linear.x`, `linear.y` (crab walk), `angular.z` |
| `/rN/odom`, `/rN/odom/tilt` | legged odometry, body roll/pitch |
| `/rN/lidar/points`, `/rN/scan` | 3D cloud; 2D scan 0.4-1.5 m above the floor |
| `/rN/camera/*`, `/rN/thermal/*`, `/rN/imu` | sensors |
| `/rN/map`, `/rN/nav_map` | SLAM map; fused navigation map |
| `/rN/mission/status` | explorer state (JSON, 1 Hz) |
| `/rN/mission/{start,stop,return_home}` | services (`std_srvs/Trigger`) |
| `/rN/mission/{takeover,resume}`, `/rN/mission/goto` | operator: manual mode and back; go to a `PointStamped` in `world` |
| `/rN/victims`, `/rN/victim_markers`, `/rN/perception/image` | detector output |
| `/rN/team/state`, `/rN/team/explored` | swarm agent: what this robot knows about the team |
| `/rN/radio/{out,in}`, `/radio/links` | radio traffic; current links |
| `/rN/ground_truth/odom` | simulator pose, used only by the radio channel and the evaluator |
| `/scenario/ground_truth` | victims including ones placed live (for the evaluator) |

TF: `world -> rN/map` (known start pose) `-> rN/odom -> rN/base_footprint` (level, at the
ground) `-> rN/base_link` (body roll/pitch) `->` legs and sensors.
