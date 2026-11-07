import math

import numpy as np

UNKNOWN, FREE, OCCUPIED = -1, 0, 100


def process_scan(ranges, angle_min, angle_increment, range_min, range_max, cap=1.0):
    if not (math.isfinite(range_min) and math.isfinite(range_max)
            and 0.0 <= range_min < range_max and cap > range_min):
        return []
    if not (math.isfinite(angle_min) and math.isfinite(angle_increment)):
        return []
    limit = min(cap, range_max)
    rays = []
    for index, value in enumerate(ranges):
        value = float(value)
        if math.isnan(value) or value < range_min:
            continue
        if math.isinf(value):
            if value > 0:
                rays.append((angle_min + index * angle_increment, limit, False))
        elif value <= range_max:
            rays.append((angle_min + index * angle_increment,
                         min(value, limit), value <= limit))
    return rays


class GridMap:
    def __init__(self, width=10.0, height=10.0, resolution=0.05, origin=(0.0, 0.0)):
        if min(width, height, resolution) <= 0:
            raise ValueError("Grid dimensions/resolution must be positive")
        self.resolution = resolution
        self.origin = tuple(origin)
        self.width = width
        self.height = height
        self.data = np.full((math.ceil(height / resolution), math.ceil(width / resolution)),
                            UNKNOWN, dtype=np.int8)

    def cell(self, x, y):
        return (math.floor((x - self.origin[0]) / self.resolution),
                math.floor((y - self.origin[1]) / self.resolution))

    def inside(self, ix, iy):
        return 0 <= iy < self.data.shape[0] and 0 <= ix < self.data.shape[1]

    def seed_robot_footprint(self, pose, radius):
        """Only the physical area currently occupied by the robot is assumed free.

        This closes the sensor's blind spot; it does not reveal surrounding space
        and never overwrites an observed obstacle.
        """
        x, y = pose[:2]
        ix, iy = self.cell(x, y)
        n = math.ceil(radius / self.resolution) + 1
        for row in range(max(0, iy - n), min(self.data.shape[0], iy + n + 1)):
            for col in range(max(0, ix - n), min(self.data.shape[1], ix + n + 1)):
                x0 = self.origin[0] + col * self.resolution
                y0 = self.origin[1] + row * self.resolution
                dx = max(abs(x0 - x), abs(x0 + self.resolution - x))
                dy = max(abs(y0 - y), abs(y0 + self.resolution - y))
                if dx * dx + dy * dy <= radius * radius and self.data[row, col] == UNKNOWN:
                    self.data[row, col] = FREE

    def update(self, rays, lidar_pose, range_min):
        lx, ly, yaw = lidar_pose
        free, occupied = set(), set()
        for angle, distance, hit in rays:
            a = angle + yaw
            dx, dy = distance * math.cos(a), distance * math.sin(a)
            end = self.cell(lx + dx, ly + dy)
            if hit and self.inside(*end):
                occupied.add(end)
            steps = max(1, math.ceil(distance / (self.resolution / 3)))
            for k in range(steps + 1):
                d = distance * k / steps
                if d < range_min:
                    continue
                ix, iy = self.cell(lx + dx * k / steps, ly + dy * k / steps)
                if not self.inside(ix, iy) or (hit and (ix, iy) == end):
                    continue
                x0 = self.origin[0] + ix * self.resolution
                y0 = self.origin[1] + iy * self.resolution
                far_x = max(abs(x0 - lx), abs(x0 + self.resolution - lx))
                far_y = max(abs(y0 - ly), abs(y0 + self.resolution - ly))
                if far_x * far_x + far_y * far_y <= distance * distance:
                    free.add((ix, iy))
        for ix, iy in free - occupied:
            self.data[iy, ix] = FREE
        for ix, iy in occupied:
            self.data[iy, ix] = OCCUPIED

    def collision_view(self, radius, margin):
        return CollisionView(self, radius + margin)


def inflate(mask, radius, resolution):
    rows, cols = mask.shape
    out = np.zeros_like(mask, dtype=bool)
    reach = math.ceil(radius / resolution) + 1
    for dy in range(-reach, reach + 1):
        for dx in range(-reach, reach + 1):
            gap_x = max(abs(dx) - 1, 0) * resolution
            gap_y = max(abs(dy) - 1, 0) * resolution
            if math.hypot(gap_x, gap_y) > radius:
                continue
            x0, x1 = max(0, dx), min(cols, cols + dx)
            y0, y1 = max(0, dy), min(rows, rows + dy)
            if x0 < x1 and y0 < y1:
                out[y0:y1, x0:x1] |= mask[y0-dy:y1-dy, x0-dx:x1-dx]
    return out


class CollisionView:
    def __init__(self, grid, radius):
        self.origin, self.resolution = grid.origin, grid.resolution
        self.width, self.height = grid.width, grid.height
        self.radius = radius
        self.optimistic = inflate(grid.data == OCCUPIED, radius, grid.resolution)
        self.strict = inflate(grid.data != FREE, radius, grid.resolution)

    def valid(self, point, allow_unknown=True):
        x, y = point[:2]
        ox, oy = self.origin
        if not (ox + self.radius < x < ox + self.width - self.radius
                and oy + self.radius < y < oy + self.height - self.radius):
            return False
        ix = math.floor((x - ox) / self.resolution)
        iy = math.floor((y - oy) / self.resolution)
        return not bool((self.optimistic if allow_unknown else self.strict)[iy, ix])

    def edge_free(self, start, end, allow_unknown=True):
        # Extra half-cell sampling plus conservative cell-square inflation.
        n = max(1, math.ceil(math.dist(start[:2], end[:2]) / (self.resolution / 2)))
        return all(self.valid((start[0] + (end[0] - start[0]) * k / n,
                               start[1] + (end[1] - start[1]) * k / n), allow_unknown)
                   for k in range(n + 1))
