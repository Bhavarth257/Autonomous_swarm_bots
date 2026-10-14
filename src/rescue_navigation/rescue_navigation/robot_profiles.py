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
    body_height=0.49,                   # body centre above the ground at the default 0.5 rad stance (measured)
    footprint=[[0.35, 0.27], [0.35, -0.27], [-0.35, -0.27], [-0.35, 0.27]],   # body 0.66, wheels 0.52 wide
    cruise=0.45,                        # tall on a short wheelbase: gentler than the rover
    min_speed=0.2,
    max_vel=[0.6, 0.0, 1.6],
    rotate_vel=1.2,
    blind_radius=1.3,                   # lidar ~0.6 m up sees the floor from ~1.3 m
    self_radius=0.45,
    body_radius=0.55,
    robot_clearance=0.6,
    return_speed=0.25,
    teammate_radius=0.4,
    teammate_height=0.45,
    scan_min_height=0.80,               # its body sits 0.43-0.55 m up, sensors to ~0.75 m
    stance_topic=True,                  # /<ns>/stance (std_msgs/Float64, rad): V half-angle
    reach_radius=0.25,                  # 0.85 m doors leave its centre ~0.35 m of slack
    step_max=0.10,                      # climbs 8-9 cm reliably, 10 cm about half the time (tested)
    world_climb_max=0.09,               # so its worlds only have debris it can get over
)
DEFAULT = 'wheelleg'


def profile(name):
    if name not in PROFILES:
        raise ValueError(f"robot must be one of {', '.join(PROFILES)}, not {name!r}")
    return PROFILES[name]
