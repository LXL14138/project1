import math
import random
import time
from bisect import bisect_right
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


def reference_path(states, collision, goal, deadline):
    if time.perf_counter() >= deadline:
        return [], []
    points = [tuple(state[:2]) for state in states]
    center = (goal[0] + goal[2] / 2, goal[1] + goal[3] / 2)
    if center != points[-1] and collision.edge_free(points[-1], center):
        points.append(center)
    reference = [points[0]]
    distances = [0.]
    index = 0
    while index < len(points) - 1:
        for next_index in range(len(points) - 1, index, -1):
            if time.perf_counter() >= deadline:
                return [], []
            if collision.edge_free(points[index], points[next_index]):
                point = points[next_index]
                if point != reference[-1]:
                    distances.append(distances[-1] + math.dist(reference[-1], point))
                    reference.append(point)
                index = next_index
                break
        else:
            # An original arc may be clear even when its chord is blocked.
            return [], []
    return reference, distances


def project_to_reference(point, reference, distances, lower, upper):
    best = (math.inf, lower)
    first = max(0, bisect_right(distances, lower) - 1)
    for index in range(first, len(reference) - 1):
        offset = distances[index]
        if offset > upper:
            break
        x, y = reference[index]
        dx, dy = reference[index + 1][0] - x, reference[index + 1][1] - y
        length = distances[index + 1] - offset
        along = ((point[0] - x) * dx + (point[1] - y) * dy) / length
        along = min(length, upper - offset, max(0., lower - offset, along))
        projection = (x + dx * along / length, y + dy * along / length)
        best = min(best, (math.dist(point, projection), offset + along))
    return best


def steering_change(controls):
    rates = [0., *(omega for _, omega in controls), 0.]
    return sum(abs(current - previous) for previous, current in zip(rates, rates[1:]))


def follow_reference(states, sequence, collision, goal, tau, controls, initial_controls, deadline):
    reference, distances = reference_path(states, collision, goal, deadline)
    if len(reference) < 2:
        return states, sequence
    guided_states, guided_controls = [states[0]], []
    progress, previous_omega = 0., 0.
    for _ in range(2 * len(sequence)):
        if time.perf_counter() >= deadline:
            break
        target_distance = min(distances[-1], progress + C.REFERENCE_LOOKAHEAD)
        index = min(bisect_right(distances, target_distance) - 1, len(reference) - 2)
        x, y = reference[index]
        dx, dy = reference[index + 1][0] - x, reference[index + 1][1] - y
        ratio = (target_distance - distances[index]) / (distances[index + 1] - distances[index])
        target = (x + ratio * dx, y + ratio * dy)
        heading = math.atan2(dy, dx)
        pose = guided_states[-1]
        candidates = []
        for v, omega in initial_controls if not guided_controls else controls:
            if v == 0 and omega == 0:
                continue
            next_pose = propagate(pose, v, omega, tau)
            cross_track, next_progress = project_to_reference(
                next_pose[:2], reference, distances, max(0., progress - 0.05),
                progress + max(0.75, abs(v) * tau))
            reached = in_goal(next_pose, goal)
            score = (1.6 * math.dist(next_pose[:2], target) + 1.1 * cross_track
                     + 0.12 * abs(wrap(next_pose[2] - heading)) + 0.03 * (v < 0)
                     - 0.20 * max(0., next_progress - progress)
                     + 0.04 * abs(omega - previous_omega))
            candidates.append((not reached, score, next_pose, (v, omega), next_progress))
        candidates.sort(key=lambda candidate: candidate[:2])
        for outside_goal, _, next_pose, control, next_progress in candidates:
            if time.perf_counter() >= deadline:
                return states, sequence
            if not trajectory_collision_free(pose, *control, collision, tau):
                continue
            guided_states.append(next_pose)
            guided_controls.append(control)
            previous_omega = control[1]
            progress = max(progress, next_progress)
            if not outside_goal:
                if steering_change(guided_controls) < steering_change(sequence):
                    return guided_states, guided_controls
                return states, sequence
            break
        else:
            break
    return states, sequence


def kinodynamic_rrt(start, collision, goal, tau=C.TAU, max_iterations=C.MAX_ITERATIONS, goal_bias=C.GOAL_SAMPLE_RATE, time_budget=C.TIME_BUDGET,
                    seed=7, controls=C.CONTROLS, initial_horizon=None):
    if not math.isfinite(tau) or tau <= 0:
        raise ValueError('tau must be positive and finite')
    begin = time.perf_counter()
    deadline = begin + time_budget
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
        initial_controls = tuple((v, w) for v, w in controls if (v != 0 or w != 0) and trajectory_collision_free(start, v, w, collision,
                                                               initial_horizon, False))
        if not initial_controls:
            return result([], [], 0, 'local_motion_blocked')
    nodes = np.empty((max_iterations + 1, 3))
    nodes[0] = start
    parents, control_used = [-1], [None]
    visited = {tuple(round(value, 4) for value in start)}
    count = 1
    for iteration in range(1, max_iterations + 1):
        if time.perf_counter() >= deadline:
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
            states, sequence = follow_reference(states, sequence, collision, goal, tau, controls, initial_controls, deadline)
            return result(states, sequence, iteration, 'success')
    return result([], [], max_iterations, 'iteration_limit')
