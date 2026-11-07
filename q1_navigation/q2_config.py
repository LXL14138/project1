MAP_ORIGIN_X, MAP_ORIGIN_Y = -5.0, -5.0
WORLD_WIDTH, WORLD_HEIGHT = 10.0, 10.0
START_X, START_Y, START_YAW = -4.0, -4.0, 0.0
GOAL = (3.65, 3.65, 0.7, 0.7)  # lower-left x, y, width, height: 0.7 m square centred at (4, 4)
SCAN_TOPIC, ODOM_TOPIC, CMD_VEL_TOPIC = '/scan', '/odom', '/cmd_vel'
MAP_FRAME = 'q2_map'
ROBOT_RADIUS = 0.25
SENSOR_RANGE = 1.0
MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED = 0.25, 1.82
GRID_RESOLUTION, SAFETY_MARGIN = 0.1, 0.03
RANDOM_SEED = 7
RETRY_PERIOD = 1.0
CONTROL_PERIOD, INPUT_TIMEOUT = 0.1, 0.8

TAU = 0.5
REFERENCE_LOOKAHEAD = 0.40  # meters along the collision-checked reference
ANGULAR_CHOICES = (-1.82, -1.46, -1.09, -0.73, -0.36, 0., 0.36, 0.73, 1.09, 1.46, 1.82)
# Forward/reverse straight and arcs, rotation in place, and stop.
CONTROLS = tuple((v, w) for v in (-MAX_LINEAR_SPEED, 0., MAX_LINEAR_SPEED)
                 for w in ANGULAR_CHOICES)
MAX_ITERATIONS = 50000
GOAL_SAMPLE_RATE = 0.15
TIME_BUDGET = 3.0
POSITION_TOLERANCE = 0.10
YAW_TOLERANCE = 0.25
LOG_DIRECTORY = '~/q2_navigation'
