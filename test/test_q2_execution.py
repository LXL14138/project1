import pytest

rclpy = pytest.importorskip('rclpy')

from q1_navigation import q2_config as K
from q1_navigation.q2_control import propagate
from q1_navigation.q2_mapping import FREE, OCCUPIED
from q1_navigation.q2_ros_node import Q2NavigationNode


@pytest.fixture
def execution(tmp_path, monkeypatch):
    monkeypatch.setattr(K, 'LOG_DIRECTORY', str(tmp_path))
    rclpy.init(args=[])
    node = Q2NavigationNode(start=(1., 1., 0.), goal=(8., 8., .5, .5))
    clock = [10.]
    commands, requests = [], []
    node.now = lambda: clock[0]
    node.command = lambda v=0., w=0.: commands.append((v, w))
    node.update_map = lambda: None
    node.grid.data[:] = FREE
    node.collision = node.grid.collision_view(.25, .03)
    node.controls = [(.25, 0.), (.25, .36)]
    node.states = [(1., 1., 0.)]
    for control in node.controls:
        node.states.append(propagate(node.states[-1], *control, K.TAU))
    node.pose = node.states[0]
    node.scan_stamp = node.odom_stamp = clock[0]

    def request(reason):
        node.clear_sequence(reason)
        requests.append(reason)

    node.plan = request
    try:
        yield node, clock, commands, requests
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


def test_each_planned_control_is_held_for_tau(execution):
    node, clock, commands, requests = execution
    node.tick()
    assert commands[-1] == (.25, 0.)
    clock[0] += .49
    node.pose = propagate(node.states[0], .25, 0., .49)
    node.scan_stamp = node.odom_stamp = clock[0]
    node.tick()
    assert node.control_index == 0 and commands[-1] == (.25, 0.)
    clock[0] = 10.5
    node.pose = node.states[1]
    node.scan_stamp = node.odom_stamp = clock[0]
    node.tick()
    assert node.control_index == 1 and commands[-1] == (.25, .36)
    assert node.control_started == 10.5 and not requests


def test_stale_input_cancels_control_sequence_and_stops(execution):
    node, clock, commands, _ = execution
    node.tick()
    clock[0] += 1.
    node.tick()
    assert commands[-1] == (0., 0.) and not node.controls
    assert node.control_started is None and node.replan_reason == 'stale_input'


def test_unknown_execution_region_stops_and_requests_replanning(execution):
    node, _, commands, requests = execution
    node.grid.data[:] = -1
    node.collision = node.grid.collision_view(.25, .03)
    node.tick()
    assert requests == ['blocked_or_unobserved_local_motion']
    assert commands[-1] == (0., 0.)


def test_pose_deviation_replans_from_feedback(execution):
    node, _, commands, requests = execution
    node.pose = (1.2, 1., 0.)
    node.tick()
    assert requests == ['execution_deviation'] and commands[-1] == (0., 0.)


def test_new_obstacle_blocks_a_future_control(execution):
    node, _, commands, requests = execution
    node.controls = [(.25, 0.)]*10
    node.states = [(1., 1., 0.)]
    for control in node.controls:
        node.states.append(propagate(node.states[-1], *control, K.TAU))
    ix, iy = node.grid.cell(2., 1.)
    node.grid.data[iy, ix] = OCCUPIED
    node.collision = node.grid.collision_view(.25, .03)
    node.tick()
    assert requests == ['future_trajectory_blocked'] and commands[-1] == (0., 0.)


def test_execution_reserves_clearance_until_input_timeout_stop(execution):
    from q1_navigation.q2_planner import trajectory_collision_free
    node, _, commands, requests = execution
    node.controls = node.controls[:1]
    node.states = node.states[:2]
    ix, iy = node.grid.cell(1.55, 1.)
    node.grid.data[iy, ix] = OCCUPIED
    node.collision = node.grid.collision_view(.25, .03)
    assert trajectory_collision_free(node.pose, .25, 0., node.collision, K.TAU)
    node.tick()
    assert requests == ['blocked_or_unobserved_local_motion']
    assert commands[-1] == (0., 0.)


def test_q2_configuration_is_independent_of_q1(tmp_path, monkeypatch):
    from q1_navigation import q1_config
    from q1_navigation.q2_ros_node import parse_arguments
    monkeypatch.setattr(q1_config, 'START_X', 99.)
    monkeypatch.setattr(q1_config, 'WORLD_WIDTH', -1.)
    monkeypatch.setattr(q1_config, 'SCAN_TOPIC', '/wrong_scan')
    monkeypatch.setattr(K, 'START_X', 2.)
    monkeypatch.setattr(K, 'GOAL', (4., 4., .4, .5))
    monkeypatch.setattr(K, 'WORLD_WIDTH', 8.)
    monkeypatch.setattr(K, 'SCAN_TOPIC', '/q2_test_scan')
    monkeypatch.setattr(K, 'LOG_DIRECTORY', str(tmp_path))
    args = parse_arguments([])
    assert args.start[0] == 2. and args.goal == K.GOAL
    rclpy.init(args=[])
    node = None
    try:
        node = Q2NavigationNode()
        assert node.start_pose[0] == 2. and node.goal == K.GOAL
        assert node.grid.width == 8.
        assert node.scan_sub.topic_name == '/q2_test_scan'
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


def test_acceleration_lag_does_not_cancel_a_control_before_tau(execution):
    node, clock, commands, requests = execution
    node.controls = [(.25, 1.82)]
    node.states = [(1., 1., 0.), propagate((1., 1., 0.), .25, 1.82, K.TAU)]
    node.tick()
    clock[0] = 10.2
    node.pose = (1.008, 1., .064)
    node.scan_stamp = node.odom_stamp = clock[0]
    node.tick()
    assert commands[-1] == (.25, 1.82) and not requests
    clock[0] = 10.5
    node.pose = (1.07, 1.02, .45)
    node.scan_stamp = node.odom_stamp = clock[0]
    node.tick()
    assert requests == ['execution_deviation'] and commands[-1] == (0., 0.)


def test_obstacle_still_interrupts_an_active_control_before_tau(execution):
    node, clock, commands, requests = execution
    node.controls = node.controls[:1]
    node.states = node.states[:2]
    node.tick()
    clock[0] = 10.1
    node.pose = propagate(node.states[0], .25, 0., .1)
    node.scan_stamp = node.odom_stamp = clock[0]
    ix, iy = node.grid.cell(1.55, 1.)
    node.grid.data[iy, ix] = OCCUPIED
    node.collision = node.grid.collision_view(.25, .03)
    node.tick()
    assert requests == ['blocked_or_unobserved_local_motion']
    assert commands[-1] == (0., 0.) and node.control_started is None


def test_feedback_error_uses_the_odometry_timestamp(execution):
    node, clock, commands, requests = execution
    node.controls = [(.25, 1.82), (.25, 0.)]
    node.states = [(1., 1., 0.)]
    for control in node.controls:
        node.states.append(propagate(node.states[-1], *control, K.TAU))
    node.tick()
    clock[0] = 10.5
    node.pose = propagate(node.states[0], .25, 1.82, .3)
    node.odom_stamp, node.scan_stamp = 10.3, clock[0]
    node.tick()
    assert node.control_index == 1 and commands[-1] == (.25, 0.) and not requests


def test_successful_plan_can_be_replaced_without_failure_cooldown(execution):
    node, clock, _, _ = execution
    node.goal = (1.6, 1., .3, .3)
    Q2NavigationNode.plan(node, 'initial')
    assert node.controls and node.attempts == 1
    clock[0] += .1
    Q2NavigationNode.plan(node, 'execution_deviation')
    assert node.controls and node.attempts == 2
    node.grid.data[:] = -1
    node.collision = node.grid.collision_view(.25, .03)
    Q2NavigationNode.plan(node, 'blocked_or_unobserved_local_motion')
    assert not node.controls and node.attempts == 3
    clock[0] += .1
    Q2NavigationNode.plan(node, 'retry')
    assert node.attempts == 3
