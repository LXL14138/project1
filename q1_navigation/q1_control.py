import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1],
            a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def anchor_transform(initial_odom, desired_start):
    yaw = wrap(desired_start[2] - initial_odom[2])
    c, s = math.cos(yaw), math.sin(yaw)
    return (desired_start[0] - c * initial_odom[0] + s * initial_odom[1],
            desired_start[1] - s * initial_odom[0] - c * initial_odom[1], yaw)


def propagate(pose, linear, angular, dt):
    x, y, yaw = pose
    half = angular * dt / 2
    sinc = 1.0 if abs(half) < 1e-9 else math.sin(half) / half
    distance = linear * dt * sinc
    return (x + distance * math.cos(yaw + half),
            y + distance * math.sin(yaw + half), wrap(yaw + angular * dt))


def command_safe(pose, linear, angular, collision, horizon):
    """Check the swept disk along the actual commanded arc in known free space."""
    n = max(1, math.ceil(horizon / 0.05))
    previous = pose
    for i in range(1, n + 1):
        current = propagate(pose, linear, angular, horizon * i / n)
        if not collision.edge_free(previous, current, allow_unknown=False):
            return False
        previous = current
    return True


def tracking_command(pose, target, cruise, max_linear, max_angular,
                     heading_gain, turn_angle, lookahead):
    distance = math.dist(pose[:2], target[:2])
    error = wrap(math.atan2(target[1] - pose[1], target[0] - pose[0]) - pose[2])
    angular = max(-max_angular, min(max_angular, heading_gain * error))
    linear = min(cruise, max_linear, distance)
    linear *= max(0.0, math.cos(error)) * min(1.0, distance / lookahead)
    if abs(error) > turn_angle:
        linear = 0.0
    return linear, angular
