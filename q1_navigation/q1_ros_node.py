import argparse
import csv
from datetime import datetime
import math
from pathlib import Path as FilePath
import sys

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from . import q1_config as C
from .q1_control import anchor_transform, command_safe, compose, tracking_command
from .q1_mapping import GridMap, process_scan
from .q1_planner import in_goal, path_length, rrt
from .run_plot import next_result_path, save_run_plot
from .q1_ros_utils import grid_to_message, quaternion_yaw, rrt_path_to_ros_path


class Q1NavigationNode(Node):
    def __init__(self, start=None, goal=None):
        super().__init__('q1_navigation')
        self.start_pose = tuple(start) if start is not None else (C.START_X, C.START_Y, C.START_YAW)
        self.goal = tuple(goal) if goal is not None else C.GOAL
        self.grid = GridMap(C.WORLD_WIDTH, C.WORLD_HEIGHT, C.GRID_RESOLUTION,
                            (C.MAP_ORIGIN_X, C.MAP_ORIGIN_Y))
        self.pose = self.offset = self.collision = self.pending_scan = None
        self.odom_frame = self.base_frame = None
        self.odom_stamp = self.scan_stamp = -math.inf
        self.path, self.waypoint = [], 1
        self.attempts, self.distance = 0, 0.0
        self.previous_pose = None
        self.last_plan_time = -math.inf
        self.finished = False
        self.trajectory, self.plans = [], []
        self.run_status = 'Interrupted'
        self.result_path = next_result_path('q1')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, C.CMD_VEL_TOPIC, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/q1/map', retained)
        self.path_pub = self.create_publisher(Path, '/q1/path', retained)
        self.odom_sub = self.create_subscription(Odometry, C.ODOM_TOPIC, self.on_odom,
                                                  qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(LaserScan, C.SCAN_TOPIC, self.on_scan,
                                                  qos_profile_sensor_data)
        log_dir = FilePath(C.LOG_DIRECTORY).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.csv')
        self.log_file = self.log_path.open('w', newline='', encoding='utf-8')
        self.log = csv.writer(self.log_file)
        self.log.writerow(['attempt', 'replan_count', 'trigger', 'runtime_s',
                           'result', 'planned_path_m', 'travelled_distance_m'])
        self.log_file.flush()
        self.timer = self.create_timer(C.CONTROL_PERIOD, self.tick)
        self.get_logger().info(f'Q1 navigation ready. Start={self.start_pose}, goal={self.goal}. '
                               f'Planning log: {self.log_path}')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def seconds(stamp):
        return stamp.sec + stamp.nanosec / 1e9

    def on_odom(self, msg):
        if self.finished or not msg.header.frame_id or not msg.child_frame_id:
            return
        stamp = self.seconds(msg.header.stamp)
        if stamp < self.odom_stamp:
            self.stop_run('Simulation reset: restart this node.')
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        odom_pose = (p.x, p.y, quaternion_yaw(q))
        if not all(math.isfinite(v) for v in odom_pose):
            return
        if self.offset is None:
            self.odom_frame, self.base_frame = msg.header.frame_id, msg.child_frame_id
            self.offset = anchor_transform(odom_pose, self.start_pose)
            t = TransformStamped()
            t.header.frame_id, t.header.stamp = C.MAP_FRAME, msg.header.stamp
            t.child_frame_id = self.odom_frame
            t.transform.translation.x, t.transform.translation.y = self.offset[:2]
            t.transform.rotation.z = math.sin(self.offset[2] / 2)
            t.transform.rotation.w = math.cos(self.offset[2] / 2)
            self.tf_broadcaster.sendTransform(t)
        elif (self.odom_frame, self.base_frame) != (msg.header.frame_id, msg.child_frame_id):
            self.stop_run('Odometry frames changed: restart this node.')
            return
        self.pose = compose(self.offset, odom_pose)
        if self.previous_pose is not None:
            self.distance += math.dist(self.pose[:2], self.previous_pose[:2])
        self.previous_pose, self.odom_stamp = self.pose, stamp
        if not self.trajectory or math.dist(self.pose[:2], self.trajectory[-1][:2]) > 0.01:
            self.trajectory.append(self.pose)

    def on_scan(self, msg):
        stamp = self.seconds(msg.header.stamp)
        if not msg.header.frame_id or stamp <= self.scan_stamp:
            return
        if (self.pending_scan is None or
                stamp - self.seconds(self.pending_scan.header.stamp) >= C.INPUT_TIMEOUT):
            self.pending_scan = msg

    def update_map(self):
        if self.pending_scan is None or self.pose is None:
            return
        scan = self.pending_scan
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, scan.header.frame_id, Time.from_msg(scan.header.stamp)).transform
        except TransformException:
            return
        rays = process_scan(scan.ranges, scan.angle_min, scan.angle_increment,
                            scan.range_min, scan.range_max, C.SENSOR_RANGE)
        self.pending_scan = None
        if not rays:
            return
        lidar_pose = compose(self.offset, (t.translation.x, t.translation.y, quaternion_yaw(t.rotation)))
        self.grid.seed_robot_footprint(self.pose, C.ROBOT_RADIUS)
        self.grid.update(rays, lidar_pose, scan.range_min)
        self.collision = self.grid.collision_view(C.ROBOT_RADIUS, C.SAFETY_MARGIN)
        self.scan_stamp = self.seconds(scan.header.stamp)
        self.map_pub.publish(grid_to_message(self.grid, C.MAP_FRAME, scan.header.stamp))

    def command(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.cmd_pub.publish(msg)

    def plan(self, reason):
        self.command()  # Stop before bounded, synchronous planning.
        if self.now() - self.last_plan_time < C.RETRY_PERIOD:
            return
        self.attempts += 1
        result = rrt(self.pose[:2], self.collision, self.goal, C.RRT_STEP,
                     C.RRT_MAX_ITERATIONS, C.RRT_GOAL_BIAS, C.RRT_TIME_BUDGET,
                     C.RANDOM_SEED + self.attempts - 1)
        self.path, self.waypoint = result.path, 1
        self.last_plan_time = self.now()
        self.log.writerow([self.attempts, self.attempts - 1, reason, result.runtime,
                           result.reason, path_length(result.path), self.distance])
        self.log_file.flush()
        if self.path:
            self.plans.append(list(self.path))
        self.path_pub.publish(rrt_path_to_ros_path(self.path, C.MAP_FRAME,
                                                   self.get_clock().now().to_msg()))
        self.get_logger().info(f'Plan {self.attempts}: {result.reason}, {result.runtime:.3f}s; '
                               f'replans={self.attempts - 1}')
        # Return to executor so fresh scan/odom callbacks run before driving.

    def tick(self):
        if self.finished:
            self.command()
            return
        self.update_map()
        if (self.pose is None or self.collision is None
                or not -.1 <= self.now() - self.odom_stamp < C.INPUT_TIMEOUT
                or not -.1 <= self.now() - self.scan_stamp < C.INPUT_TIMEOUT):
            self.command()
            return
        if in_goal(self.pose, self.goal):
            self.stop_run('Goal reached')
            return
        if not self.path:
            self.plan('initial' if self.attempts == 0 else 'retry')
            return
        while (self.waypoint < len(self.path) and
               math.dist(self.pose[:2], self.path[self.waypoint]) < C.WAYPOINT_TOLERANCE):
            self.waypoint += 1
        if self.waypoint == len(self.path):
            self.plan('path_finished_outside_goal')
            return
        remaining = [self.pose[:2]] + self.path[self.waypoint:]
        if not all(self.collision.edge_free(a, b) for a, b in zip(remaining, remaining[1:])):
            self.plan('path_blocked')
            return
        linear, angular = tracking_command(self.pose, self.path[self.waypoint], C.CRUISE_SPEED,
                                           C.MAX_LINEAR_SPEED, C.MAX_ANGULAR_SPEED,
                                           C.HEADING_GAIN, C.TURN_IN_PLACE_ANGLE, C.LOOKAHEAD)
        for scale in (1.0, 0.5, 0.25):
            if command_safe(self.pose, linear * scale, angular, self.collision,
                            C.INPUT_TIMEOUT + 2 * C.CONTROL_PERIOD):
                self.command(linear * scale, angular)
                return
        self.plan('blocked_or_unobserved_local_motion')

    def stop_run(self, reason):
        self.finished = True
        self.command()
        self.run_status = reason
        self.save_plot()
        self.get_logger().info(f'{reason}; replans={max(0, self.attempts-1)}; '
                               f'travelled={self.distance:.3f}m')

    def save_plot(self):
        try:
            save_run_plot(self.result_path, 'q1', self.grid, self.collision,
                          self.start_pose, self.goal, C.ROBOT_RADIUS, self.trajectory,
                          self.plans, self.run_status, max(0, self.attempts-1), self.distance)
            self.get_logger().info(f'Run plot saved: {self.result_path}')
        except Exception as error:  # never let plotting break shutdown
            self.get_logger().error(f'Could not save run plot: {error}')

    def close(self):
        self.save_plot()
        if rclpy.ok():
            self.command()
        self.log_file.close()
        self.get_logger().info(f'Final replans={max(0, self.attempts-1)}, '
                               f'travelled={self.distance:.3f}m. CSV: {self.log_path}')


def parse_arguments(args=None):
    parser = argparse.ArgumentParser(description='Q1 geometric RRT navigation')
    parser.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        default=(C.START_X, C.START_Y, C.START_YAW),
                        help='Initial map pose; yaw is in radians')
    parser.add_argument('--goal', nargs=4, type=float, metavar=('X', 'Y', 'WIDTH', 'HEIGHT'),
                        default=C.GOAL, help='Goal rectangle in map coordinates')
    argv = sys.argv if args is None else [parser.prog, *args]
    options = parser.parse_args(remove_ros_args(args=argv)[1:])
    if not all(math.isfinite(value) for value in (*options.start, *options.goal)):
        parser.error('Start and goal values must be finite numbers')
    if options.goal[2] <= 0 or options.goal[3] <= 0:
        parser.error('Goal width and height must be positive')
    return options


def main(args=None):
    options = parse_arguments(args)
    # Keep the context alive on Ctrl+C so the finally block can send zero speed.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Q1NavigationNode(start=options.start, goal=options.goal)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()
