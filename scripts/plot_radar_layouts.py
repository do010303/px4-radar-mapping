#!/usr/bin/env python3
"""Pictures of the radar layouts of scripts/radar_layouts.py, for the README.

One row per layout: on the left the x500 with each radar's field of view drawn as a wedge out to
0.6 m (ISK 122 x 41 deg, AOP 75 x 104 deg), on the right a map of all directions around the vehicle (azimuth x elevation, base_link)
coloured by the radar that sees them, grey where none does, with the directions that decided the
flights marked.

    python3 scripts/plot_radar_layouts.py               # docs/img/radar_layouts.png
    python3 scripts/plot_radar_layouts.py 6 cube out.png
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from plotstyle import FAINT, GRID, INK, INK2, SURFACE
from radar_layouts import CANDIDATES, LAYOUTS, coverage, fov, mounts

WEDGE = 0.6                 # m, drawn length of a field of view
# Categorical slots 1-6 (validated: scripts of the dataviz palette, light surface); each wedge is labelled.
COLOURS = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300")
TITLES = {3: "x500_radar3: vòng ngang", 6: "x500_radar6: 3 × 3", "cube": "6 ISK lập phương (đề xuất)",
          "aop": "x500_radar_aop: 1 ISK + 5 AOP (đề xuất)"}
MARKS = [   # (azimuth, elevation, label): directions in base_link
    (0.0, 0.0, "phía trước"),
    (0.0, 29.0, "phía trước khi\nchúi mũi 29°"),
    (0.0, -90.0, "ngay dưới bụng"),
    (0.0, 90.0, "đỉnh"),
]


def fov_surface(name, r_mount, p_mount, n=13):
    az, el = fov(name)
    a = np.radians(np.linspace(-az, az, n))
    e = np.radians(np.linspace(-el, el, n))
    A, E = np.meshgrid(a, e)
    local = np.stack((np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)), axis=-1) * WEDGE
    pts = local @ r_mount.T + p_mount
    return pts[..., 0], pts[..., 1], pts[..., 2]


def draw_vehicle(ax):
    # x500: arms at 45 deg, rotors 0.25 m out on them, a flat body.
    for k in range(4):
        t = math.radians(45 + 90 * k)
        c = np.array([0.25 * math.cos(t), 0.25 * math.sin(t), 0.02])
        ax.plot([0, c[0]], [0, c[1]], [0, c[2]], color=INK2, lw=2)
        s = np.linspace(0, 2 * math.pi, 40)
        ax.plot(c[0] + 0.12 * np.cos(s), c[1] + 0.12 * np.sin(s), np.full(40, c[2]), color=FAINT, lw=1)

    ax.quiver(0, 0, 0, 0.45, 0, 0, color=INK, lw=1.5, arrow_length_ratio=0.15)
    ax.text(0.5, 0, 0.03, "mũi (x)", color=INK, fontsize=8)


def sky(layout, n_az=361, n_el=181):
    """Index of the first radar seeing each direction of an azimuth x elevation grid, -1 for none."""
    az = np.radians(np.linspace(-180, 180, n_az))
    el = np.radians(np.linspace(-90, 90, n_el))
    A, E = np.meshgrid(az, el)
    d = np.stack((np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)), axis=-1)
    who = np.full(A.shape, -1)

    for i, (name, r, _) in enumerate(mounts(layout)):
        az_half, el_half = fov(name)
        local = d @ r
        seen = (np.abs(np.degrees(np.arctan2(local[..., 1], local[..., 0]))) <= az_half) & \
               (np.abs(np.degrees(np.arcsin(np.clip(local[..., 2], -1, 1)))) <= el_half) & (local[..., 0] > 0)
        who[(who < 0) & seen] = i

    return who


def plot(layouts, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import mpl_toolkits
    from matplotlib.colors import ListedColormap

    # A pip matplotlib next to the distro's: take the mplot3d that belongs to the matplotlib imported.
    own = os.path.join(os.path.dirname(os.path.dirname(matplotlib.__file__)), "mpl_toolkits")
    if os.path.isdir(own) and own not in list(mpl_toolkits.__path__):
        mpl_toolkits.__path__.insert(0, own)

    from matplotlib.projections import register_projection
    from mpl_toolkits.mplot3d import Axes3D

    register_projection(Axes3D)     # pyplot's own attempt failed on the distro's copy

    fig = plt.figure(figsize=(13, 4.6 * len(layouts)), facecolor=SURFACE)

    for row, layout in enumerate(layouts):
        ms = mounts(layout)
        ax = fig.add_subplot(len(layouts), 2, 2 * row + 1, projection="3d", facecolor=SURFACE)
        draw_vehicle(ax)

        for i, (name, r, p) in enumerate(ms):
            x, y, z = fov_surface(name, r, p)
            ax.plot_surface(x, y, z, color=COLOURS[i], alpha=0.35, linewidth=0, shade=False)

            for corner in ((0, 0), (0, -1), (-1, 0), (-1, -1)):
                ax.plot([p[0], x[corner]], [p[1], y[corner]], [p[2], z[corner]], color=COLOURS[i], lw=0.8)

            tip = p + r[:, 0] * WEDGE * 1.12
            ax.text(*tip, name, color=INK, fontsize=8, ha="center")

        ax.set_xlim(-0.7, 0.7), ax.set_ylim(-0.7, 0.7), ax.set_zlim(-0.7, 0.7)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=22, azim=-58)
        ax.set_xlabel("x trước (m)", color=INK2, fontsize=8)
        ax.set_ylabel("y trái (m)", color=INK2, fontsize=8)
        ax.set_zlabel("z lên (m)", color=INK2, fontsize=8)
        ax.tick_params(colors=INK2, labelsize=7)

        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.pane.set_facecolor(SURFACE)
            axis.pane.set_edgecolor(GRID)

        title = TITLES.get(layout, str(layout))
        ax.set_title(f"{title}: {100 * coverage(layout):.0f}% mọi hướng", color=INK, fontsize=11, loc="left")

        # Sky map: who sees each direction.
        sx = fig.add_subplot(len(layouts), 2, 2 * row + 2, facecolor=SURFACE)
        who = sky(layout)
        cmap = ListedColormap(["#e4e3df", *COLOURS[:len(ms)]])
        sx.imshow(who + 1, origin="lower", extent=(-180, 180, -90, 90), cmap=cmap, vmin=0, vmax=len(ms),
                  aspect="auto", interpolation="nearest")

        for i, (name, r, _) in enumerate(ms):
            b = r[:, 0]
            a, e = math.degrees(math.atan2(b[1], b[0])), math.degrees(math.asin(np.clip(b[2], -1, 1)))
            # Below the boresight (above it for the one looking down), clear of the marked directions.
            a, e = (a, e - 10) if e > -80 else (0, -72)
            if e > 70:
                a, e = 60, 78
            sx.text(a, e, name, color=INK, fontsize=8, ha="center", va="center",
                    bbox=dict(boxstyle="round,pad=0.2", fc=SURFACE, ec="none", alpha=0.85))

        for a, e, label in MARKS:
            sx.plot(a, e, marker="o", ms=8, mfc="none", mec=INK, mew=1.5)
            sx.annotate(label, (a, e), xytext=(10, -12 if e > 0 else 6), textcoords="offset points",
                        color=INK, fontsize=7.5)

        sx.set_xticks(range(-180, 181, 45)), sx.set_yticks(range(-90, 91, 30))
        sx.tick_params(colors=INK2, labelsize=8)
        sx.set_xlabel("azimuth (°, 0 = mũi, + = trái)", color=INK2, fontsize=8)
        sx.set_ylabel("elevation (°)", color=INK2, fontsize=8)
        sx.set_title("radar nào thấy hướng nào (xám: không radar nào)", color=INK2, fontsize=9, loc="left")

        for s in sx.spines.values():
            s.set_visible(False)

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=110, facecolor=SURFACE)
    print(f"wrote {path}")


def main():
    args = sys.argv[1:]
    path = args.pop() if args and args[-1].endswith(".png") else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "docs", "img", "radar_layouts.png")
    layouts = [int(a) if a.isdigit() else a for a in args] or [*LAYOUTS, *CANDIDATES]
    plot(layouts, os.path.normpath(path))


if __name__ == "__main__":
    main()
