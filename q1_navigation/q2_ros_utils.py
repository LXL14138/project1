import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


def quaternion_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rrt_path_to_ros_path(path, frame_id, stamp):
    msg = Path()
    msg.header.frame_id, msg.header.stamp = frame_id, stamp
    for i, point in enumerate(path):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x, pose.pose.position.y = float(point[0]), float(point[1])
        yaw = 0.0
        if len(point) >= 3:
            yaw = point[2]
        elif i + 1 < len(path):
            yaw = math.atan2(path[i+1][1] - point[1], path[i+1][0] - point[0])
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.poses.append(pose)
    return msg


def grid_to_message(grid, frame_id, stamp):
    msg = OccupancyGrid()
    msg.header.frame_id, msg.header.stamp = frame_id, stamp
    msg.info.resolution = float(grid.resolution)
    msg.info.width, msg.info.height = grid.data.shape[1], grid.data.shape[0]
    msg.info.origin.position.x, msg.info.origin.position.y = grid.origin
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.data.ravel().tolist()
    return msg
