import math

import numpy as np

from q1_navigation.q1_control import anchor_transform, command_safe, compose, propagate
from q1_navigation.q1_mapping import FREE, OCCUPIED, UNKNOWN, GridMap, process_scan
from q1_navigation.q1_planner import in_goal, rrt


def test_scan_cap_invalid_and_no_hit():
    rays = process_scan([.5, 3., math.inf, math.nan, -math.inf, .01, 20.],
                        0., .1, .12, 10., 1.)
    assert len(rays) == 3
    assert [ray[1:] for ray in rays] == [(.5, True), (1., False), (1., False)]


def test_scan_exact_range_is_hit():
    assert process_scan([1.0], 0, .1, .1, 5)[0][2]


def test_scan_bad_metadata_is_rejected():
    assert process_scan([1], 0, .1, .1, math.nan) == []


def test_no_free_or_hit_information_beyond_one_meter():
    grid = GridMap(4, 4, .05)
    rays = process_scan([5.] * 360, 0, math.pi / 180, .12, 10)
    grid.update(rays, (2, 2, 0), .12)
    assert not np.any(grid.data == OCCUPIED)
    rows, cols = np.where(grid.data == FREE)
    assert all(math.hypot((x + .5)*.05 - 2, (y + .5)*.05 - 2) < 1 for y, x in zip(rows, cols))
    assert grid.data[40, 65] == UNKNOWN


def test_hit_occludes_cells_and_later_ray_can_clear_dynamic_obstacle():
    grid = GridMap(4, 4, .05)
    grid.update([(0, .5, True)], (1, 1, 0), .1)
    ix, iy = grid.cell(1.5, 1)
    assert grid.data[iy, ix] == OCCUPIED
    assert grid.data[iy, ix + 2] == UNKNOWN
    grid.update([(0, 1., False)], (1, 1, 0), .1)
    assert grid.data[iy, ix] == FREE


def test_scan_hit_wins_over_free_evidence_same_scan():
    grid = GridMap(4, 4, .05)
    grid.update([(0, .5, True), (0, 1., False)], (1, 1, 0), .1)
    ix, iy = grid.cell(1.5, 1)
    assert grid.data[iy, ix] == OCCUPIED


def test_lidar_rotation_and_translation():
    grid = GridMap(4, 4, .05)
    grid.update([(0, .5, True)], (1.2, 1.1, math.pi / 2), .1)
    ix, iy = grid.cell(1.2, 1.6)
    assert grid.data[iy, ix] == OCCUPIED


def test_robot_footprint_seed_cannot_erase_obstacle_or_reveal_surroundings():
    grid = GridMap(4, 4, .05)
    grid.data[40, 40] = OCCUPIED
    grid.seed_robot_footprint((2, 2, 0), .25)
    assert grid.data[40, 40] == OCCUPIED
    assert grid.data[40, 50] == UNKNOWN
    assert np.count_nonzero(grid.data == FREE) > 0


def test_unknown_planning_allowed_execution_forbidden():
    view = GridMap(4, 4, .05).collision_view(.25, .03)
    assert view.valid((2, 2))
    assert not view.valid((2, 2), False)
    assert not command_safe((2, 2, 0), .15, 0, view, 1.)


def test_inflation_respects_robot_disk_and_boundaries():
    grid = GridMap(4, 4, .05)
    grid.data[:] = FREE
    grid.data[40, 40] = OCCUPIED
    view = grid.collision_view(.25, .03)
    assert not view.valid((2.2, 2.02))
    assert view.valid((2.5, 2.02))
    assert not view.valid((.27, 2))
    assert not view.edge_free((1, 2), (3, 2))


def test_negative_map_origin():
    grid = GridMap(10, 10, .05, (-5., -5.))
    assert grid.cell(-4.9, -4.9) == (1, 1) or grid.cell(-4.9, -4.9) == (2, 2)
    assert grid.collision_view(.25, .03).valid((-4, -4))


def test_anchor_handles_nonzero_initial_yaw():
    odom = (3., -2., .7)
    desired = (1., 1., -1.2)
    assert np.allclose(compose(anchor_transform(odom, desired), odom), desired)


def test_exact_differential_drive_and_zero_angular_speed():
    assert np.allclose(propagate((0, 0, 0), .25, 0, .4), (.1, 0, 0))
    assert np.allclose(propagate((0, 0, 0), 1., 1., math.pi/2), (1, 1, math.pi/2))


def test_curved_command_checks_arc_not_only_endpoint():
    grid = GridMap(5, 5, .05)
    grid.data[:] = FREE
    ix, iy = grid.cell(2.7, 2.3)
    grid.data[iy, ix] = OCCUPIED
    view = grid.collision_view(.25, .03)
    assert not command_safe((2, 2, 0), 1, 1, view, math.pi/2)
    assert command_safe((1, 1, 0), .15, .2, view, 1)


def test_rrt_reaches_goal_region_and_avoids_wall():
    grid = GridMap(5, 5, .05)
    grid.data[:] = FREE
    grid.data[0:65, 48:52] = OCCUPIED
    view = grid.collision_view(.25, .03)
    goal = (4., 1., .5, .5)
    result = rrt((1, 1), view, goal, time_budget=3., seed=12)
    assert result.path, result.reason
    assert in_goal(result.path[-1], goal)
    assert all(view.edge_free(a, b) for a, b in zip(result.path, result.path[1:]))


def test_unreachable_goal_returns_within_budget():
    grid = GridMap(5, 5, .05)
    grid.data[:, 48:52] = OCCUPIED
    result = rrt((1, 1), grid.collision_view(.25, .03), (4, 1, .5, .5), time_budget=.05)
    assert not result.path
    assert result.runtime < .5


def test_goal_is_not_hardcoded_to_nine_nine():
    view = GridMap(4, 4, .05).collision_view(.25, .03)
    goal = (.7, 2.5, .4, .4)
    result = rrt((1, 1), view, goal, time_budget=2., seed=4)
    assert result.path and in_goal(result.path[-1], goal)
