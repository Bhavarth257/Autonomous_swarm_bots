"""The wheel-leg robot's dimensions and tuning, in one place (launch files, Nav2 config,
mapping, exploration and perception all read it).

Four driven wheels on two free rockers (one per side, linked by a differential); a motor
per side opens/closes the rocker's V, which sets the stance height and levels the body.
"""
ROBOT = {
    'xacro': 'wheelleg.urdf.xacro',
    'spawn_z': 0.6,
    'body_height': 0.43,                # body centre above the ground at the default 0.8 rad stance
    'lidar_height': 0.54,               # lidar above the ground (body centre + 0.11)
    'stance_topic': True,               # /<ns>/stance (std_msgs/Float64, rad): V half-angle
    # Nav2
    'footprint': [[0.40, 0.30], [0.40, -0.30], [-0.40, -0.30], [-0.40, 0.30]],   # body 0.66, wheels 0.79 long x 0.60 wide
    # a gentle fall-off: the cheapest path runs down the middle of doors and corridors
    # instead of hugging the wall at half a body width (the planner treats the robot as a
    # point, so corners of the outline swinging round a jamb were scraping)
    'inflation': 1.0,
    'cost_scaling': 1.8,                # how fast the cost falls off away from walls
    'cruise': 0.45,                     # tall on a short wheelbase: gentle
    'min_speed': 0.2,
    'max_vel': [0.6, 0.0, 1.6],         # x, y, yaw for the velocity smoother
    'rotate_vel': 1.2,
    'wall_guard': True,                 # wall_guard.py between Nav2 and the wheels
    # The global costmap takes obstacles from nav_map only: scan marks far from the robot
    # never clear (nothing raytraces through them), and a teammate caught in the scan once
    # sealed the entrance for the trip home. The local costmap still uses the scan.
    'global_scan': False,
    # mapping (traversability)
    'step_max': 0.10,                   # climbs 8-9 cm reliably, 10 cm about half the time (tested)
    'world_climb_max': 0.09,            # so its worlds only have debris it can get over
    'blind_radius': 1.3,                # lidar ~0.6 m up sees the floor from ~1.3 m
    'self_radius': 0.45,
    'body_radius': 0.55,                # lidar points within this of the body are the robot itself
    # teammates' corners are 0.5 m from their centre and gossiped positions lag up to
    # ~0.45 m: a smaller radius left their bodies in the height map as ghosts in doorways
    'robot_clearance': 0.9,
    'trail_radius': 0.36,               # own path stays walkable for its whole width (Nav2 keeps the
                                        # centre 0.30 m from anything lethal)
    # exploration
    'return_speed': 0.25,               # average speed home through rubble, for the early return
    'reach_radius': 0.28,               # 0.85 m doors leave its centre ~0.29 m of slack
    # perception
    'teammate_radius': 0.4,
    'teammate_height': 0.40,            # body centre above the floor
    'scan_min_height': 1.0,             # SLAM scan slice starts above a teammate: sensors reach ~0.70 m,
                                        # ~0.85 m on a block or seen while pitched; at 0.80 teammates
                                        # got into the scan and sealed doorways
}
