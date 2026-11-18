"""Shared look of the example's plots: colours by job, recessive axes, obstacle outlines."""

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
FAINT = "#b9b8b2"
GRID = "#e4e3df"

# Detection labels (scene.classify): one fixed colour each.
ON_OBSTACLE = "#2a78d6"
GROUND = "#eb6834"
UNEXPLAINED = "#8a8984"


def figure(rows, height_ratios, size):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(rows, 1, figsize=size, facecolor=SURFACE,
                             gridspec_kw={"height_ratios": height_ratios})
    return plt, fig, axes


def style_axes(ax, xlabel=None, ylabel=None):
    ax.set_facecolor(SURFACE)
    ax.grid(color=GRID, lw=0.6)
    ax.tick_params(colors=INK2)

    if xlabel:
        ax.set_xlabel(xlabel, color=INK2)

    if ylabel:
        ax.set_ylabel(ylabel, color=INK2)

    for s in ax.spines.values():
        s.set_visible(False)


def scatter_labels(ax, points, labels, col, size=4):
    """Detections coloured by label; points are world (or sensor) xyz, col the y-axis column."""
    for mask, colour, alpha, label in ((labels.ground, GROUND, 0.25, "Ground return"),
                                       (labels.unexplained, UNEXPLAINED, 0.45, "Unexplained"),
                                       (~labels.ground & ~labels.unexplained, ON_OBSTACLE, 0.4, "On an obstacle")):
        ax.scatter(points[mask, 0], points[mask, col], s=size, color=colour, alpha=alpha,
                   linewidths=0, label=label)


def draw_obstacles(plt, top, side, obstacles, label=lambda o: True):
    """Obstacle outlines: footprint in the top view, extent in the side view."""
    import numpy as np

    for o in obstacles:
        (x, y, z), (sx, sy, sz), yaw = o.centre, o.size, o.yaw
        c, s = np.cos(yaw), np.sin(yaw)
        corners = np.array([[-sx, -sy], [sx, -sy], [sx, sy], [-sx, sy], [-sx, -sy]]) / 2.0
        xy = corners @ np.array([[c, s], [-s, c]]) + (x, y)
        top.plot(xy[:, 0], xy[:, 1], color=FAINT, lw=1.5, zorder=0)

        if label(o):
            top.annotate(o.name, (x, y), xytext=(6, 6), textcoords="offset points", color=INK2, fontsize=8)

        half = max(abs(c * sx) + abs(s * sy), 0.1) / 2.0
        side.add_patch(plt.Rectangle((x - half, z - sz / 2), 2 * half, sz, fill=False, ec=FAINT, lw=1.5))


def title(ax, text):
    ax.set_title(text, color=INK, loc="left", fontsize=12)


def legend(ax, loc="upper right"):
    ax.legend(frameon=False, labelcolor=INK, fontsize=9, loc=loc, markerscale=2)
