from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException

from q1_navigation import q1_config, q2_config
from q1_navigation.q1_ros_node import Q1NavigationNode
from q1_navigation.q2_ros_node import Q2NavigationNode


def scan_at(ns):
    scan = LaserScan()
    scan.header.frame_id = 'base_scan'
    scan.header.stamp = Time(nanoseconds=ns).to_msg()
    return scan


@pytest.mark.parametrize('NavigationNode', [Q1NavigationNode, Q2NavigationNode])
def test_scan_waits_for_its_transform_instead_of_being_overwritten(NavigationNode):
    rclpy.init(args=[])
    try:
        buffer = Buffer()
        fixed = TransformStamped()
        fixed.header.frame_id = 'base_footprint'
        fixed.child_frame_id = 'base_scan'
        fixed.transform.rotation.w = 1.
        buffer.set_transform_static(fixed, 'test')
        state = SimpleNamespace(scan_stamp=-float('inf'), pending_scan=None,
                                seconds=NavigationNode.seconds)
        mapped = []
        for frame in range(20):
            ns = 1_000_000_000 + frame * 100_000_000
            pose = TransformStamped()
            pose.header.frame_id = 'odom'
            pose.child_frame_id = 'base_footprint'
            pose.header.stamp = Time(nanoseconds=ns - 10_000_000).to_msg()
            pose.transform.rotation.w = 1.
            buffer.set_transform(pose, 'test')
            NavigationNode.on_scan(state, scan_at(ns))
            buffer.lookup_transform('odom', 'base_scan', Time())
            try:
                buffer.lookup_transform('odom', 'base_scan',
                                        Time.from_msg(state.pending_scan.header.stamp))
            except TransformException:
                continue
            state.scan_stamp = state.seconds(state.pending_scan.header.stamp)
            mapped.append(state.scan_stamp)
            state.pending_scan = None
        assert len(mapped) >= 8
        assert all(a < b for a, b in zip(mapped, mapped[1:]))
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('NavigationNode, C', [(Q1NavigationNode, q1_config),
                                               (Q2NavigationNode, q2_config)])
def test_expired_pending_scan_is_replaced(NavigationNode, C):
    state = SimpleNamespace(scan_stamp=-float('inf'), pending_scan=None,
                            seconds=NavigationNode.seconds)
    first = scan_at(1_000_000_000)
    NavigationNode.on_scan(state, first)
    NavigationNode.on_scan(state, scan_at(1_100_000_000))
    assert state.pending_scan is first
    fresh = scan_at(1_000_000_000 + int((C.INPUT_TIMEOUT + .1) * 1e9))
    NavigationNode.on_scan(state, fresh)
    assert state.pending_scan is fresh
