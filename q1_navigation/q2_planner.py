import math
import random
import time
from dataclasses import dataclass

import numpy as np

from . import q2_config as C
from .q2_control import propagate, wrap


def in_goal(point, goal):
    x, y, w, h = goal
    return x <= point[0] <= x + w and y <= point[1] <= y + h



@dataclass
class KinodynamicResult:
    states: list
    controls: list
    runtime: float
    iterations: int
    reason: str


def sample_point(rng, collision, goal, goal_bias):
    if rng.random() < goal_bias:
        return (rng.uniform(goal[0], goal[0] + goal[2]),
                rng.uniform(goal[1], goal[1] + goal[3]))
    ox, oy = collision.origin
    r = collision.radius + 1e-5
    return (rng.uniform(ox + r, ox + collision.width - r),
            rng.uniform(oy + r, oy + collision.height - r))


def nearest_node(nodes, point):
    delta = nodes[:, :2] - point
    return int(np.argmin(np.einsum('ij,ij->i', delta, delta)))


def trajectory_collision_free(state, v, omega, collision, duration=C.TAU, allow_unknown=True):
    checks = max(1, math.ceil(duration / 0.05))
    previous = state
    for i in range(1, checks + 1):
        current = propagate(state, v, omega, duration * i / checks)
        if not collision.edge_free(previous, current, allow_unknown):
            return False
        previous = current
    return True


def best_control(nearest, point, collision, tau=C.TAU, controls=C.CONTROLS):
    candidates = []
    for v, omega in controls:
        if v == 0 and omega == 0:
            continue
        state = propagate(nearest, v, omega, tau)
        candidates.append((math.dist(state[:2], point), state, (v, omega)))
    candidates.sort(key=lambda candidate: candidate[0])
    for _, state, control in candidates:
        if trajectory_collision_free(nearest, *control, collision, tau):
            return state, control
    return None, None


def control_path_length(controls, tau=C.TAU):
    return sum(abs(v) * tau for v, _ in controls)


def dense_trajectory(states, controls, tau=C.TAU):
    if not states:
        return []
    result = [states[0]]
    checks = max(1, math.ceil(tau / 0.05))
    for state, (v, omega) in zip(states, controls):
        result.extend(propagate(state, v, omega, tau * i / checks)
                      for i in range(1, checks + 1))
    return result


def kinodynamic_rrt(start, collision, goal, tau=C.TAU, max_iterations=C.MAX_ITERATIONS,
                    goal_bias=C.GOAL_SAMPLE_RATE, time_budget=C.TIME_BUDGET,
                    seed=7, controls=C.CONTROLS, initial_horizon=None):
    if not math.isfinite(tau) or tau <= 0:
        raise ValueError('tau must be positive and finite')
    begin = time.perf_counter()
    rng = random.Random(seed)
    start = (float(start[0]), float(start[1]), wrap(start[2]))

    def result(states, sequence, iterations, reason):
        return KinodynamicResult(states, sequence, time.perf_counter() - begin, iterations, reason)

    if not collision.valid(start):
        return result([], [], 0, 'start_blocked')
    if in_goal(start, goal):
        return result([start], [], 0, 'already_in_goal')
    initial_controls = controls
    if initial_horizon is not None:
        initial_controls = tuple((v, w) for v, w in controls if (v != 0 or w != 0)
                                 and trajectory_collision_free(start, v, w, collision,
                                                               initial_horizon, False))
        if not initial_controls:
            return result([], [], 0, 'local_motion_blocked')
    nodes = np.empty((max_iterations + 1, 3))
    nodes[0] = start
    parents, control_used = [-1], [None]
    visited = {tuple(round(value, 4) for value in start)}
    count = 1
    for iteration in range(1, max_iterations + 1):
        if time.perf_counter() - begin >= time_budget:
            return result([], [], iteration - 1, 'time_budget')
        point = sample_point(rng, collision, goal, goal_bias)
        nearest = nearest_node(nodes[:count], point)
        candidates = initial_controls if nearest == 0 else controls
        new, chosen = best_control(nodes[nearest], point, collision, tau, candidates)
        if new is None:
            continue
        key = tuple(round(value, 4) for value in new)
        if key in visited:
            continue
        visited.add(key)
        nodes[count] = new
        parents.append(nearest)
        control_used.append(chosen)
        count += 1
        if in_goal(new, goal):
            states, sequence = [], []
            current = count - 1
            while current >= 0:
                states.append(tuple(nodes[current]))
                if control_used[current] is not None:
                    sequence.append(control_used[current])
                current = parents[current]
            states.reverse()
            sequence.reverse()
            return result(states, sequence, iteration, 'success')
    return result([], [], max_iterations, 'iteration_limit')
