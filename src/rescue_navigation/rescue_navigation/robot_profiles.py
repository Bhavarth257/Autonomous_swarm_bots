"""What differs between the two rescue robots, in one place (launch files, Nav2 config,
mapping, exploration and perception all read it). Choose with robot:=wheelleg|rover|hexapod
(default wheelleg).

  rover     six 28 cm wheels, skid steer, 0.88 x 0.55 m, 0.7 m/s, climbs 12 cm steps
  hexapod   18-DOF spider, tripod gait, 0.35 m/s, crab-walks, climbs 12 cm steps
  wheelleg  four driven wheels on two free rockers (one per side, linked by a differential),
            a motor per side opens/closes the rocker's V: stance height and levelling
"""
PROFILES = {
    'rover': {
        'xacro': 'rover.urdf.xacro',
        'spawn_z': 0.2,
        'body_height': 0.14,            # base_link (axles) above the ground
        # Nav2
        'footprint': [[0.45, 0.29], [0.45, -0.29], [-0.45, -0.29], [-0.45, 0.29]],
        'robot_radius': None,
        'inflation': 0.75,
        'cost_scaling': 3.5,            # how fast the cost falls off away from walls
        'cruise': 0.7,
        'min_speed': 0.25,
        'max_vel': [0.8, 0.0, 1.8],     # x, y, yaw for the velocity smoother
        'rotate_vel': 1.5,
        'holonomic': False,
        # mapping (traversability)
        'step_max': 0.13,               # climbs 12 cm (tested), not 14
        'world_climb_max': 0.12,        # climbable debris the world generator may use
        'blind_radius': 0.8,            # lidar on the deck at 0.33 m sees the floor from ~0.7 m
        'self_radius': 0.5,
        'body_radius': 0.6,             # lidar points within this of the body are the robot itself
        'robot_clearance': 0.7,         # teammates' bodies are not obstacles
        # exploration
        'return_speed': 0.3,            # average speed home through rubble, for the early return
        'reach_radius': 0.29,           # half its width: where its centre can go
        # perception
        'teammate_radius': 0.45,
        'teammate_height': 0.25,        # body centre above the floor
        'scan_min_height': 0.40,        # SLAM scan slice starts above a teammate's height
    },
    'hexapod': {
        'xacro': 'hexabot.urdf.xacro',
        'spawn_z': 0.25,
        'body_height': 0.15,
        'footprint': None,
        'robot_radius': 0.22,           # the legs (span ~0.53 x 0.58 m) may brush door jambs
        'inflation': 0.7,
        'cost_scaling': 3.5,
        'cruise': 0.35,                 # below the gait's 0.45 cap so turns stay smooth
        'min_speed': 0.2,
        'max_vel': [0.45, 0.25, 1.8],
        'rotate_vel': 1.6,
        'holonomic': True,              # crab walks
        'step_max': 0.14,
        'world_climb_max': 0.12,
        'blind_radius': 0.7,
        'self_radius': 0.35,
        'body_radius': 0.55,
        'robot_clearance': 0.6,
        'return_speed': 0.15,
        'reach_radius': 0.20,
        'teammate_radius': 0.3,
        'teammate_height': 0.2,
        'scan_min_height': 0.40,
    },
}
PROFILES['wheelleg'] = dict(
    PROFILES['rover'],
    xacro='wheelleg.urdf.xacro',
    spawn_z=0.6,
    body_height=0.43,                   # body centre above the ground at the default 0.8 rad stance
    footprint=[[0.40, 0.30], [0.40, -0.30], [-0.40, -0.30], [-0.40, 0.30]],   # body 0.66, wheels 0.79 long x 0.60 wide
    # a gentle fall-off: the cheapest path runs down the middle of doors and corridors
    # instead of hugging the wall at half a body width (the planner treats the robot as a
    # point, so corners of the outline swinging round a jamb were scraping)
    inflation=1.0,
    cost_scaling=1.8,
    cruise=0.45,                        # tall on a short wheelbase: gentler than the rover
    min_speed=0.2,
    max_vel=[0.6, 0.0, 1.6],
    rotate_vel=1.2,
    blind_radius=1.3,                   # lidar ~0.6 m up sees the floor from ~1.3 m
    self_radius=0.45,
    body_radius=0.55,
    # teammates' corners are 0.5 m from their centre and gossiped positions lag up to
    # ~0.45 m: a smaller radius left their bodies in the height map as ghosts in doorways
    robot_clearance=0.9,
    return_speed=0.25,
    teammate_radius=0.4,
    teammate_height=0.40,
    scan_min_height=1.0,                # sensors reach ~0.70 m, ~0.85 m on a block or seen while pitched:
                                        # at 0.80 teammates got into the scan and sealed doorways
    stance_topic=True,                  # /<ns>/stance (std_msgs/Float64, rad): V half-angle
    reach_radius=0.28,                  # 0.85 m doors leave its centre ~0.29 m of slack
    wall_guard=True,                    # wall_guard.py between Nav2 and the wheels
    # The global costmap takes obstacles from nav_map only: scan marks far from the robot
    # never clear (nothing raytraces through them), and a teammate caught in the scan once
    # sealed the entrance for the trip home. The local costmap still uses the scan.
    global_scan=False,
    trail_radius=0.36,                  # own path stays walkable for its whole width (Nav2 keeps the
                                        # centre 0.30 m from anything lethal)
    lidar_height=0.54,                  # lidar above the ground (body centre + 0.11)
    step_max=0.10,                      # climbs 8-9 cm reliably, 10 cm about half the time (tested)
    world_climb_max=0.09,               # so its worlds only have debris it can get over
)
DEFAULT = 'wheelleg'


def profile(name):
    if name not in PROFILES:
        raise ValueError(f"robot must be one of {', '.join(PROFILES)}, not {name!r}")
    return PROFILES[name]
