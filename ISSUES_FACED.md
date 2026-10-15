# Issues faced

The major problems hit while building this simulation, what caused each one, and how it
was solved, grouped by area. The full step-by-step history is on the
[`Testing`](https://github.com/Bhavarth257/Autonomous_swarm_bots/tree/Testing) branch.

## 1. Choosing the robot

The robot changed four times before the wheel-leg design.

| robot | why it was tried | why it was dropped |
|---|---|---|
| Small diff-drive robot | Simplest start; Nav2 is built for it | Slow and couldn't climb any rubble |
| RHex-style hexabot (6 rotating C-legs) | Fast (0.7 m/s) and climbs 13 cm | The first legs made it bounce and backflip; fixed by reshaping the legs so the body rolls at constant height, slowing down when tilted and lengthening the body. Replaced later by a spider design that looked like a real hobby kit. |
| 18-DOF spider hexapod | Real walking, looked like a hobby kit | Slow (0.35-0.4 m/s), slow turns, legs caught door jambs, the bobbing body shook the lidar, and leg odometry drifted. Three hexapods (54 joints) were also the heaviest part of the simulation. |
| Six-wheel rover | Wheel odometry, fast, Nav2-friendly | No suspension to keep the wheels down or the body level on rubble; kept only as a comparison |
| Wheel-leg robot (final) | Rocker legs keep all four wheels on the ground; one motor per side sets the stance height and levels the body | Climbs 10 cm rather than 12 cm, so its worlds only use debris up to 9 cm |

## 2. Wheel-leg robot physics

| problem | cause | fix |
|---|---|---|
| The robot would not turn in place at all, at any tyre grip | In the first stance the wheels were 0.69 m apart front to back but only 0.48 m side to side. A skid-steer robot that long for its width can't turn, because sideways tyre scrub beats the drive torque. | Hubs moved to the centre and the stance closed until the wheelbase is no longer than the track. The drive also spins the wheels 1.8x faster when turning, to make up for scrub. |
| Stance commands barely moved the legs (1 cm instead of 13 cm) | Gazebo's stock skid-steer drive held the front and rear wheels at the same speed, so the tyres pinned the feet while the legs tried to open or close | A custom drive plugin (`RescueWheelLegDrive`) turns each wheel to match its leg's motion |
| Leg motors stopped responding once the differential was added | In Gazebo Fortress, a joint with any force or wrench command ignores velocity commands | The differential became a velocity servo: the left rocker is free and the right one mirrors it |
| Hip joints locked at spawn | A joint that starts outside its limits stays locked | Widened the range to include the starting pose |
| Turning in place rolled the robot over | Wheel traction twisted the two rockers opposite ways (wind-up). Changing joint limits at runtime to lock them only while turning did not work in Fortress. | Rocker travel limited to ±0.15 rad, plus a roll guard: past 11° it slows to a crawl and stops turning until the body is level |
| Wheels chattered | Torque-controlled wheels oscillate in Fortress | Wheels are velocity-controlled |
| Couldn't climb 12 cm steps | With one motor per side both legs of a V move together, so it can't lift one front wheel over a step | "Leg-assisted climbing" (open the V to shove the front wheels up the step) was built and tested. It reared the robot up and flipped it backwards (up to 86°), so it's switched off (`climb_attempts 0`). The world generator caps climbable debris at 9 cm, so every room stays reachable. |
| Nose-up flips on steps | Short wheelbase under a tall body | Wider default stance (0.8 rad: wheels 0.52 m apart, body 0.43 m up) and a pitch guard that stops driving further in the tipping direction past 20° |

## 3. Mapping and localisation

| problem | cause | fix |
|---|---|---|
| Robots avoided smoke rooms as if they were walls | The lidar hit smoke particles. In Fortress only the `ParticleEmitter` model plugin passes a scatter ratio to the lidar (a plain `<particle_emitter>` always gets 0.65), and the plugin reads 0 as "unset". | Smoke uses the plugin with a tiny, non-zero ratio. Readings blocked by smoke fell from 97 % to 7 %; the RGB camera is still blinded and thermal still works. A unit test guards it. |
| Doubled, rotated walls in the map ("hallucinating") | The simulated gyro drifted up to 9° a minute while a robot stood still, and SLAM can't correct heading until the robot moves | Odometry now models a gyro calibrated at standstill: ~0.02°/s residual bias |
| Python nodes fell behind on TF and missed exact-time lookups | Every robot's leg and wheel transforms flood `/tf`, and a Python TF buffer can't keep up | `BodyPose`/`FrameWatch` read time-stamped odometry and tilt straight off the topics; only the slow `map -> odom` and fixed sensor mounts come from TF |
| A robot was boxed in at the start and never moved | Teammates squeezing past were mapped right next to it, inside the lidar's blind zone, where nothing ever clears them | The SLAM scan starts above a teammate's height; teammates' positions are cleared from the height map; a stuck robot forgets obstacles in its blind zone and steps away |
| The entrance looked sealed on the way home | Teammates caught in the scan at the start left marks far from the robot that never clear | The global costmap takes obstacles from `nav_map` only, not the raw scan; the scan slice was raised to 1.0 m, above the teammates' camera heads |
| The robot's own way back was closed by ghost obstacles | Its own path was kept walkable only 0.15 m wide, but Nav2 keeps the centre of a 0.60 m robot 0.30 m from anything lethal | The trail is kept walkable 0.36 m wide |
| Nav2 couldn't plan from where the robot stood | SLAM's map only grows where the scan hits something, so it could end right under a robot in the open | `nav_map` covers a fixed area: the building outline plus the start, with a margin |

## 4. Navigation and exploration

| problem | cause | fix |
|---|---|---|
| The body's corners scraped door jambs | NavFn plans for a point and the steep cost fall-off made paths hug walls | A gentle cost fall-off (1.0 m radius) keeps paths in the middle of doors and corridors; a new `wall_guard` node checks the 3D lidar every cycle and steers or backs off |
| The controller stopped the robot in every doorway (~40 % of control cycles) | With a rectangular footprint, Regulated Pure Pursuit's collision check fires whenever the outline touches the inflated band around a wall | That check is off; the centred paths and `wall_guard` handle contact |
| 10-15 s pauses at every narrow door | Nav2's default recovery spins 1.57 rad (fails in a doorway), backs up 0.3 m at 0.05 m/s, then waits 5 s | A custom behaviour tree that clears the costmaps, backs up 0.2 m and waits 1 s, with three retries instead of six, so unreachable frontiers fail fast and get blacklisted |
| Paths dropped and the robot backed away for no reason | Under load SLAM's `map -> odom` transform came late, and the controller aborted at once ("Controller patience exceeded") | The controller rides out up to 1 s of late transforms. Dropped paths per robot went from 52/16/8 to 0/0/1. |
| A robot sat still all mission | A startup race: Nav2 sometimes failed to come up when everything started in the same second | The explorer configures and activates Nav2 itself instead of a lifecycle manager, retries, and restarts it if goals keep being rejected. Robots also start a few seconds apart. |
| A wedged robot turned into the jambs | Its escape move turned in place, swinging the long body into the door frame | The escape drives straight out along the body, towards whichever side is clearer |
| A stuck robot's escape move never ran | Every new goal reset its "am I stuck?" timer | A goal that fails without the robot moving counts as a failure |
| Robots were still deep in the building at the time limit | They explored until time ran out | Each robot heads home early, based on its distance from the entrance |

## 5. Victim detection

| problem | cause | fix |
|---|---|---|
| Victims placed ~1 m too close at 5 m range (3/6 found, 1.1 m mean error) | The detector assumed the floor 5 cm higher than it is, used the robot's own footing (raised on rubble), and placed the box's near edge instead of the body's middle | Uses the true building floor and allows for the body-centre offset. Mean error went to 0.3-0.4 m and victims found to 6/6 on the hard world. |
| A teammate reported as a victim | YOLO sometimes takes a distant dark robot for a person | Person boxes that cover a teammate's position are dropped |
| GPU memory | A detector per robot would need three CUDA contexts next to Gazebo on a 4 GB GPU | One detector process for the whole team, with a separate tracker and topics per robot |

## 6. Swarm coordination

| problem | cause | fix |
|---|---|---|
| Robots duplicated each other's work | The radio (12 m line of sight, 4 m through walls at first) kept them out of contact most of the time, and fixed left/middle/right strips didn't follow progress | Rooms are split at the start (every robot computes the same split without talking) and handed over through radio gossip; the range was raised to 25 m / 8 m |
| Robots ignored their own rooms and crowded into the same ones | A room counted as "done" at 85 % seen by any lidar, which happened through doorways long before anyone looked in with a camera (all rooms "done" by 305 s of a 672 s mission). Ownership was only a soft 5 m penalty. | Ownership holds until a hand-over; robots take frontiers in their own rooms strictly first; a room is done only at ≥ 90 % explored and when no robot reports a frontier there. Time spent in own rooms went from 0-34 % to 54-65 %, and a seed-4 mission dropped from 14.2 to 11.1 min. |
| A tipped or stuck robot's rooms were never searched | Rooms stayed assigned to it | Each robot reports its own health (down: tipped past 60° for 5 s; stuck: < 0.5 m in 2 min). Teammates split its open rooms, nearest first. A robot that is only out of radio range is not marked down. |

## 7. ROS, Gazebo and the machine

| problem | cause | fix |
|---|---|---|
| RViz crashed from the VS Code terminal (`undefined symbol __libc_pthread_init`) | Snap-packaged VS Code leaks its GTK library paths, and RViz loaded the snap's `libpthread` | The launch files remove those variables |
| "No ROS2 environment active" in VS Code | The `ros2env` extension only knows Docker-based ROS | Harmless; disable the extension |
| A planner or SLAM node never answered at startup | FastDDS shared memory left a node blocked on a port lock | A localhost-only FastDDS profile over UDP, selected when `install/setup.bash` is sourced |
| Only 6 of ~50 nodes found each other | The Wi-Fi network filtered multicast discovery | The same profile: unicast discovery on 127.0.0.1 |
| The clock froze when Wi-Fi dropped | Gazebo transport discovered peers on the network interface | `IGN_IP=127.0.0.1` in the launch file |
| Robots lost messages after many force-killed runs | Stale FastDDS shared-memory files | `fastdds shm clean` with nothing running (in Troubleshooting) |
| The simulation ran at 0.3-0.4x and SLAM fell behind | After suspend the NVIDIA unified-memory module dropped (`CUDA unknown error`) and YOLO ran on the CPU | `sudo rmmod nvidia_uvm && sudo modprobe nvidia_uvm`, or reboot |
| A robot never started in long test sessions | The laptop reached 95-98 °C and odometry arrived too late | Let it cool between runs (in Troubleshooting) |
| `colcon build` broke after installing YOLO | `pip install ultralytics` pulled in setuptools 84 | Removed it so the system's setuptools 59.6 is used; NumPy pinned below 2 for ROS Humble |
| `pytest` failed to start | A user-installed pytest 9 clashes with ROS's launch_testing plugin | Tests run with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` |
| A Gazebo server kept running after the launch ended | The server can outlive a launch that shuts itself down | The launch kills its own world's server on shutdown |
| Batch missions were always cut short | `run_batch` killed each run after the time limit + 400 s of real time, but the simulation runs at ~0.4x | A generous timeout; a mission ends by itself once every robot is home |
| Arrow keys in the teleop were mixed up (↑ read as "turn left") | The arrow's escape sequence was split up | The teleop reads raw keyboard bytes |

## 8. Still open

- **Tip-overs at door jambs.** One robot in one verification run tipped over beside a jamb
  on its way home. The likely cause is that the explorer's escape moves skip `wall_guard`,
  so a straight escape can push the front wheels up a jamb. A possible fix is to send
  escape moves through the guard's front check and give walls less grip than rubble.
- **Time-limited coverage.** In a large hard building the team turns back before every
  room is searched.
- **Distant sightings** are placed less accurately than close ones.
