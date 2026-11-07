"""Save a per-run trajectory figure to ~/run_result/<q>_pilot<N>.jpg."""
import math
from pathlib import Path
import re

import matplotlib
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Rectangle
import numpy as np

matplotlib.use('Agg')  # headless: no display needed
import matplotlib.pyplot as plt  # noqa: E402,I100,I202

RESULT_DIRECTORY = Path('~/run_result').expanduser()


def next_result_path(question):
    """Reserve the next free <question>_pilot<N>.jpg name."""
    RESULT_DIRECTORY.mkdir(parents=True, exist_ok=True)
    pattern = re.compile(rf'^{re.escape(question)}_pilot(\d+)\.jpg$')
    used = [int(m.group(1)) for p in RESULT_DIRECTORY.iterdir()
            if (m := pattern.match(p.name))]
    return RESULT_DIRECTORY / f'{question}_pilot{max(used, default=0) + 1}.jpg'


def save_run_plot(out_path, question, grid, collision, start_pose, goal, robot_radius,
                  trajectory, plans, status, replans, distance):
    """Draw map, inflated obstacles, start/goal regions, plans and driven path."""
    FREE, OCCUPIED = 0, 100
    # 0 unknown, 1 free, 2 inflation zone, 3 observed obstacle
    layer = np.where(grid.data == FREE, 1, 0)
    if collision is not None:
        layer = np.where(collision.optimistic, 2, layer)
    layer = np.where(grid.data == OCCUPIED, 3, layer)
    cmap = ListedColormap(['#d9d9d9', '#ffffff', '#f4a582', '#000000'])
    ox, oy = grid.origin
    extent = (ox, ox + grid.data.shape[1] * grid.resolution,
              oy, oy + grid.data.shape[0] * grid.resolution)

    fig, ax = plt.subplots(figsize=(8, 8), dpi=120)
    ax.imshow(layer, origin='lower', extent=extent, cmap=cmap, vmin=0, vmax=3,
              interpolation='nearest')

    gx, gy, gw, gh = goal
    ax.add_patch(Rectangle((gx, gy), gw, gh, facecolor='#4daf4a', alpha=0.45,
                           edgecolor='#1b7a1b', linewidth=2))
    sx, sy, syaw = start_pose
    ax.add_patch(Circle((sx, sy), robot_radius, facecolor='#377eb8', alpha=0.4,
                        edgecolor='#174a7e', linewidth=2))
    ax.annotate('', xy=(sx + 0.4 * math.cos(syaw), sy + 0.4 * math.sin(syaw)),
                xytext=(sx, sy), arrowprops={'arrowstyle': '->', 'color': '#174a7e', 'lw': 2})

    for path in plans[:-1]:
        if len(path) > 1:
            xs, ys = zip(*[p[:2] for p in path])
            ax.plot(xs, ys, color='#984ea3', alpha=0.25, linewidth=1)
    if plans and len(plans[-1]) > 1:
        xs, ys = zip(*[p[:2] for p in plans[-1]])
        ax.plot(xs, ys, '--', color='#984ea3', linewidth=1.5)
    if len(trajectory) > 1:
        xs, ys = zip(*[p[:2] for p in trajectory])
        ax.plot(xs, ys, color='#e41a1c', linewidth=2)
        ax.plot(xs[-1], ys[-1], 'o', color='#e41a1c')

    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    inflation = collision.radius if collision is not None else float('nan')
    ax.set_title(f'{out_path.stem}  |  {status}\n'
                 f'replans={replans}, travelled={distance:.2f} m, '
                 f'inflation radius={inflation:.2f} m')
    ax.legend(handles=[
        Patch(facecolor='#377eb8', alpha=0.4, edgecolor='#174a7e', label='start region'),
        Patch(facecolor='#4daf4a', alpha=0.45, edgecolor='#1b7a1b', label='goal region'),
        Patch(facecolor='#000000', label='observed obstacle'),
        Patch(facecolor='#f4a582', label=f'inflation ({inflation:.2f} m)'),
        Patch(facecolor='#d9d9d9', label='unknown'),
        Line2D([], [], color='#984ea3', linestyle='--', label='last planned path'),
        Line2D([], [], color='#984ea3', alpha=0.25, label='earlier plans'),
        Line2D([], [], color='#e41a1c', linewidth=2, label='driven trajectory'),
    ], loc='upper left', fontsize=8, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_path, format='jpg', pil_kwargs={'quality': 92})
    plt.close(fig)
    return out_path
