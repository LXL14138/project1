import math
import random
import time
from dataclasses import dataclass
import numpy as np

def in_goal(point, goal):
    x, y, w, h = goal
    return x <= point[0] <= x + w and y <= point[1] <= y + h

def path_length(path):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(path, path[1:]))

@dataclass
class PlanResult:
    path: list
    runtime: float
    iterations: int
    reason: str


def rrt(start, collision, goal, step=0.4, max_iterations=12000, goal_bias=0.15, time_budget=1.5, seed=7):
    begin = time.perf_counter()
    rng = random.Random(seed)
    start = tuple(start[:2])
    def result(path, iteration, reason):
        return PlanResult(path, time.perf_counter() - begin, iteration, reason)
    if not collision.valid(start):
        return result([], 0, "start_blocked")
    if in_goal(start, goal):
        return result([start], 0, "already_in_goal")
    nodes = np.empty((max_iterations + 1, 2), dtype=float)
    nodes[0] = start
    parents = [-1]
    ox, oy = collision.origin
    r = collision.radius + 1e-5
    count = 1
    for iteration in range(1, max_iterations + 1):
        if time.perf_counter() - begin >= time_budget:
            return result([], iteration - 1, "time_budget")
        if rng.random() < goal_bias:
            target = (rng.uniform(goal[0], goal[0] + goal[2]),
                      rng.uniform(goal[1], goal[1] + goal[3]))
        else:
            target = (rng.uniform(ox + r, ox + collision.width - r),
                      rng.uniform(oy + r, oy + collision.height - r))

        
        delta = nodes[:count] - target
        nearest = int(np.argmin(np.einsum('ij,ij->i', delta, delta)))
        parent = nodes[nearest]
        d = math.dist(parent, target)
        if d < 1e-8:
            continue
        scale = min(step / d, 1.0)
        new = (float(parent[0] + scale * (target[0] - parent[0])),
               float(parent[1] + scale * (target[1] - parent[1])))
        if not collision.edge_free(parent, new):
            continue
        nodes[count] = new
        parents.append(nearest)
        count += 1
        if in_goal(new, goal):
            path, current = [], count - 1
            while current >= 0:
                path.append(tuple(nodes[current]))
                current = parents[current]
            path.reverse()
            # Greedy shortcuts retain full disk collision checks and deadline.
            short = [path[0]]
            i = 0
            while i < len(path) - 1:
                j = len(path) - 1
                while j > i + 1:
                    if time.perf_counter() - begin >= time_budget:
                        return result(short + path[i + 1:], iteration, "success")
                    if collision.edge_free(path[i], path[j]):
                        break
                    j -= 1
                short.append(path[j])
                i = j
            return result(short, iteration, "success")
    return result([], max_iterations, "iteration_limit")
