import math

import numpy as np

from q1_navigation import q2_config as K
from q1_navigation.q2_control import propagate
from q1_navigation.q2_mapping import FREE, OCCUPIED, GridMap
from q1_navigation.q2_planner import in_goal
from q1_navigation.q2_planner import (
    control_path_length, dense_trajectory, kinodynamic_rrt, trajectory_collision_free,
)


def test_control_sequence_replays_every_returned_state_and_reaches_goal():
    grid = GridMap()
    grid.data[0:110, 95:105] = OCCUPIED
    view = grid.collision_view(.25, .03)
    goal = (9., 9., .7, .7)
    result = kinodynamic_rrt((1., 1., .3), view, goal, seed=7)
    assert result.reason == 'success'
    assert len(result.states) == len(result.controls)+1
    assert in_goal(result.states[-1], goal)
    for state, next_state, (v, w) in zip(result.states, result.states[1:], result.controls):
        half = w*K.TAU/2
        sinc = 1. if abs(half) < 1e-8 else math.sin(half)/half
        expected = (state[0]+K.TAU*v*sinc*math.cos(state[2]+half),
                    state[1]+K.TAU*v*sinc*math.sin(state[2]+half))
        assert np.allclose(expected, next_state[:2])
        assert np.allclose(propagate(state, v, w, K.TAU), next_state)
        assert (v, w) in K.CONTROLS and (v, w) != (0., 0.)
        assert trajectory_collision_free(state, v, w, view)
    assert control_path_length(result.controls) == len(result.controls)*.25*K.TAU
    dense = dense_trajectory(result.states, result.controls)
    assert np.allclose(dense[-1], result.states[-1])


def test_collision_along_arc_is_checked_even_when_endpoints_are_clear():
    grid = GridMap(5, 5, .05)
    grid.data[:] = FREE
    ix, iy = grid.cell(2.7, 2.3)
    grid.data[iy, ix] = OCCUPIED
    view = grid.collision_view(.25, .03)
    start = (2., 2., 0.)
    assert view.valid(start) and view.valid(propagate(start, 1., 1., math.pi/2))
    assert not trajectory_collision_free(start, 1., 1., view, math.pi/2)


def test_unknown_allowed_for_planning_but_not_execution():
    view = GridMap().collision_view(.25, .03)
    assert trajectory_collision_free((1., 1., 0.), .25, .36, view)
    assert not trajectory_collision_free((1., 1., 0.), .25, .36, view, allow_unknown=False)


def test_negative_origin_and_custom_goal():
    grid = GridMap(origin=(-5., -5.))
    goal = (-2.5, -3.8, .4, .4)
    result = kinodynamic_rrt((-4., -4., 0.), grid.collision_view(.25, .03), goal, seed=3)
    assert result.states and in_goal(result.states[-1], goal)


def test_unreachable_kinodynamic_goal_obeys_time_budget():
    grid = GridMap(5, 5, .05)
    grid.data[:, 48:52] = OCCUPIED
    result = kinodynamic_rrt((1., 1., 0.), grid.collision_view(.25, .03),
                             (4., 1., .4, .4), time_budget=.05)
    assert not result.controls and result.reason == 'time_budget'
    assert result.runtime < .5


def test_start_in_goal_has_no_controls_and_zero_length():
    result = kinodynamic_rrt((1., 1., 0.), GridMap().collision_view(.25, .03), (.9, .9, .3, .3))
    assert result.reason == 'already_in_goal'
    assert len(result.states) == 1 and result.controls == []
    assert control_path_length(result.controls) == 0


def test_first_control_is_executable_in_known_free_space():
    grid = GridMap()
    grid.data[:, :30] = FREE
    view = grid.collision_view(.25, .03)
    start = (1., 1., 0.)
    assert not trajectory_collision_free(start, .25, 0., view, 1., False)
    result = kinodynamic_rrt(start, view, (3., 1., .4, .4), initial_horizon=1.)
    assert result.reason == 'success'
    assert trajectory_collision_free(start, *result.controls[0], view, 1., False)


def test_unknown_first_motion_returns_explicit_blocked_result():
    result = kinodynamic_rrt((1., 1., 0.), GridMap().collision_view(.25, .03),
                             (3., 1., .4, .4), initial_horizon=1.)
    assert result.reason == 'local_motion_blocked' and not result.controls
    assert result.iterations == 0
