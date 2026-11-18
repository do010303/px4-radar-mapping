#!/usr/bin/env python3
"""The map of a flight as a point cloud, in a 3D viewer (Open3D), from the files the processing wrote:
<tag>.map.ply (every radar detection), <tag>.objects/*.ply (each object's points) and <tag>.objects.yaml.

    scripts/survey_process.py out/survey/live_6      # first: <tag>.map.ply
    scripts/survey_objects.py out/survey/live_6      #        <tag>.objects.yaml and <tag>.objects/
    scripts/view_map.py out/survey/live_6            # the viewer
    scripts/view_map.py out/survey/live_6 --color height --ground   # every point by height, ground included
    scripts/view_map.py out/survey/live_6 --export   # <tag>.map.color.ply (NED), with colours
    scripts/view_map.py out/survey/live_6 --export --up   # <tag>.map.color_up.ply: z up, for CloudCompare / MeshLab
    scripts/view_map.py out/survey/live_6 --png view.png            # a picture, without a window

The files are in PX4's local NED frame (scripts/frames.py: x north, y east, z down, from EKF2's origin). The
viewer shows them with up as up: east across, north ahead, up = -z; the axes drawn at the origin are
labelled N, E, D. Mouse: left drag turns, wheel zooms, shift + left drag (or middle) pans.

Colours (--color object, the default): poles blue, wires orange, blocks green (each object's sharp point cloud,
<name>.sharp.ply: --all for all its points; the rest of them hidden, --halo shows them grey), the other points
grey by height.
--color height: all by height; --color snr: by signal strength; --color radar: one colour per radar.
"""

import argparse
import glob
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
TYPE_COLOUR = {"pole": (0.16, 0.47, 0.84), "wire": (0.88, 0.39, 0.16), "box": (0.23, 0.60, 0.36)}
RADAR_COLOUR = np.array([(0.90, 0.30, 0.25), (0.25, 0.55, 0.90), (0.30, 0.75, 0.35), (0.95, 0.70, 0.20),
                         (0.65, 0.40, 0.85), (0.20, 0.75, 0.80)])
GROUND_BAND = 0.6           # m above the ground: left out unless --ground


def read_ply(path):
    """(columns: names, data (n, k), the header's frame comment) of an ASCII PLY written by survey_process."""
    names, frame = [], ""
    with open(path) as f:
        n_header = 0
        for line in f:
            n_header += 1
            if line.startswith("comment frame"):
                frame = line[len("comment frame"):].strip()
            elif line.startswith("property"):
                names.append(line.split()[-1])
            elif line.startswith("end_header"):
                break
    data = np.loadtxt(path, skiprows=n_header, ndmin=2)
    return names, data, frame


def to_view(ned):
    """NED (x north, y east, z down) -> the viewer's (east, north, up), so that up is up on screen."""
    ned = np.asarray(ned, float)
    return np.column_stack((ned[..., 1], ned[..., 0], -ned[..., 2])) if ned.ndim == 2 else \
        np.array([ned[1], ned[0], -ned[2]])


def height_colours(up, lo=None, hi=None, dim=False):
    lo = np.percentile(up, 1) if lo is None else lo
    hi = np.percentile(up, 99) if hi is None else hi
    t = np.clip((up - lo) / max(hi - lo, 1e-6), 0, 1)[:, None]
    # dark blue-grey (low) to light (high): reads as relief. dim: the points of no object (ghosts, strays),
    # dark, so that they do not hide the objects' colours.
    if dim:
        return (1 - t) * np.array([0.16, 0.18, 0.24]) + t * np.array([0.40, 0.42, 0.46])
    return (1 - t) * np.array([0.22, 0.27, 0.36]) + t * np.array([0.85, 0.86, 0.88])


def load(tag, color, with_ground, all_points=False, halo=False):
    names, data, frame = read_ply(tag + ".map.ply")
    if "px4_local_ned" not in frame:
        sys.exit(f"{tag}.map.ply is not in PX4's local NED frame ({frame or 'no frame comment'}): "
                 f"rerun scripts/survey_process.py {tag}")
    col = {n: i for i, n in enumerate(names)}
    xyz = data[:, :3]
    meta = yaml.safe_load(open(tag + ".objects.yaml")) if os.path.exists(tag + ".objects.yaml") else {}
    ground_d = meta.get("ground_d")
    if not with_ground and ground_d is not None:
        keep = xyz[:, 2] < ground_d - GROUND_BAND           # D smaller = higher
        xyz, data = xyz[keep], data[keep]

    view = to_view(xyz)
    if color == "snr":
        s = data[:, col["snr"]]
        colours = height_colours(s)
    elif color == "radar":
        colours = RADAR_COLOUR[data[:, col["radar"]].astype(int) % len(RADAR_COLOUR)]
    else:
        colours = height_colours(view[:, 2], dim=color == "object")

    layers = [(view, colours)]
    if color == "object":
        # Each object's points in its colour, and out of the grey layer (drawn twice, the grey hid them).
        key = lambda xyz: {tuple(r) for r in np.round(xyz, 2)}  # noqa: E731
        taken = set()
        # Each object's sharp point cloud (survey_objects.sharpen: poles and wires without the far detections'
        # halo, which stays grey), or with all_points every point of it.
        paths = [p for p in sorted(glob.glob(os.path.join(tag + ".objects", "*.ply")))
                 if p.endswith(".sharp.ply") != all_points]
        if not all_points and not paths:
            paths = sorted(glob.glob(os.path.join(tag + ".objects", "*.ply")))     # written before .sharp.ply
        for path in paths:
            kind = os.path.basename(path).split("_")[0]
            _, d, _ = read_ply(path)
            if len(d):
                layers.append((to_view(d[:, :3]), np.tile(TYPE_COLOUR.get(kind, (0.9, 0.9, 0.9)), (len(d), 1))))
                taken |= key(d[:, :3])
        if not all_points and not halo:
            # The halo (an object's points its sharp cloud left out) hides the cloud inside it: out too.
            for path in glob.glob(os.path.join(tag + ".objects", "*.ply")):
                if not path.endswith(".sharp.ply"):
                    _, d, _ = read_ply(path)
                    if len(d):
                        taken |= key(d[:, :3])
        rest = np.fromiter((tuple(r) not in taken for r in np.round(xyz, 2)), bool, len(xyz))
        layers[0] = (view[rest], colours[rest])
    return layers, meta


def object_labels(meta):
    """(position in the viewer, text) per object of <tag>.objects.yaml (NED)."""
    out = []
    for o in meta.get("objects", []):
        if o["type"] == "pole":
            p = np.array(o["p"], float)
            at = to_view(p - [0, 0, o["h"] + 0.8])
            text = f"{o['name']}  N {p[0]:.1f} E {p[1]:.1f}  h {o['h']:.1f}"
        elif o["type"] == "wire":
            a, b = np.array(o["p0"], float), np.array(o["p1"], float)
            at = to_view((a + b) / 2 - [0, 0, 0.8])
            text = f"{o['name']}  N {a[0]:.1f}..{b[0]:.1f} E {a[1]:.1f}..{b[1]:.1f} D {(a[2] + b[2]) / 2:.1f}"
        else:
            c = np.array(o["c"], float)
            at = to_view(c - [0, 0, o["size"][2] / 2 + 0.8])
            text = f"{o['name']}  N {c[0]:.1f} E {c[1]:.1f} D {c[2]:.1f}"
        out.append((at, text))
    return out


def geometries(layers, meta):
    import open3d as o3d

    geoms = []
    for i, (xyz, rgb) in enumerate(layers):
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
        pc.colors = o3d.utility.Vector3dVector(rgb)
        geoms.append({"name": "map" if i == 0 else f"object {i}", "geometry": pc})

    # PX4's local NED axes at the origin (N red, E green, D blue), 5 m.
    axes = o3d.geometry.LineSet()
    ends = np.array([to_view([0, 0, 0]), to_view([5, 0, 0]), to_view([0, 5, 0]), to_view([0, 0, 5])])
    axes.points = o3d.utility.Vector3dVector(ends)
    axes.lines = o3d.utility.Vector2iVector([[0, 1], [0, 2], [0, 3]])
    axes.colors = o3d.utility.Vector3dVector([(1, 0.2, 0.2), (0.2, 0.9, 0.2), (0.3, 0.5, 1.0)])
    geoms.append({"name": "NED axes", "geometry": axes})

    # A 10 m grid on the ground.
    gd = meta.get("ground_d", 0.0)
    xs = np.arange(-30, 81, 10.0)
    ys = np.arange(-30, 31, 10.0)
    pts, lines = [], []
    for x in xs:
        lines.append([len(pts), len(pts) + 1]); pts += [(x, ys[0], -gd), (x, ys[-1], -gd)]
    for y in ys:
        lines.append([len(pts), len(pts) + 1]); pts += [(xs[0], y, -gd), (xs[-1], y, -gd)]
    grid = o3d.geometry.LineSet(o3d.utility.Vector3dVector(pts), o3d.utility.Vector2iVector(lines))
    grid.paint_uniform_color((0.35, 0.35, 0.38))
    geoms.append({"name": "ground grid (10 m)", "geometry": grid})
    return geoms


def export(tag, layers, up=False):
    """<tag>.map.color.ply: binary, with colours, in PX4's local NED frame (as the files it came from), or with
    up: in the viewer's axes (x east, y north, z up, from the same origin), which other viewers (CloudCompare,
    MeshLab: z up) show the right way up: <tag>.map.color_up.ply."""
    import open3d as o3d

    xyz = np.vstack([l[0] for l in layers])
    rgb = np.vstack([l[1] for l in layers])
    out = xyz if up else np.column_stack((xyz[:, 1], xyz[:, 0], -xyz[:, 2]))
    path = tag + (".map.color_up.ply" if up else ".map.color.ply")
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(out))
    pc.colors = o3d.utility.Vector3dVector(rgb)
    o3d.io.write_point_cloud(path, pc, write_ascii=False)
    print(f"wrote {path} ({len(out)} points, " + ("x east, y north, z up" if up else "PX4 local NED")
          + " from EKF2's origin, with colours)")


def snapshot(path, geoms, point_size=3.0, size=(1600, 1000)):
    """A picture from the south-west, above (the line running east across, the pylons standing up): Open3D's
    classic viewer in a hidden window (its offscreen renderer gives black images on some GPUs)."""
    import open3d as o3d

    vis = o3d.visualization.Visualizer()
    vis.create_window(width=size[0], height=size[1], visible=False)
    for g in geoms:
        vis.add_geometry(g["geometry"])
    opt = vis.get_render_option()
    opt.background_color = np.array([0.08, 0.08, 0.1])
    opt.point_size = point_size
    ctl = vis.get_view_control()
    ctl.set_lookat([30.0, 0.0, 4.0])
    ctl.set_front([-0.35, -0.75, 0.55])
    ctl.set_up([0.0, 0.0, 1.0])
    ctl.set_zoom(0.45)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(path, do_render=True)
    vis.destroy_window()
    print(f"wrote {path}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("tag", help="out/survey/<tag> (without extension)")
    p.add_argument("--color", choices=("object", "height", "snr", "radar"), default="object")
    p.add_argument("--ground", action="store_true", help="show the ground's points too")
    p.add_argument("--all", action="store_true", help="colour every point of each object, not only its sharp "
                                                      "point cloud")
    p.add_argument("--halo", action="store_true", help="show, grey, the points of each object its sharp point cloud "
                                                       "leaves out (hidden by default: they hide the cloud)")
    p.add_argument("--point-size", type=float, default=3.0)
    p.add_argument("--export", action="store_true", help="write <tag>.map.color.ply (NED) and exit")
    p.add_argument("--up", action="store_true", help="with --export: x east, y north, z up instead "
                                                     "(<tag>.map.color_up.ply, the right way up in CloudCompare)")
    p.add_argument("--png", help="render a picture to this file instead of opening a window")
    args = p.parse_args()

    for need in (".map.ply",):
        if not os.path.exists(args.tag + need):
            sys.exit(f"no {args.tag}{need}: run scripts/survey_process.py {args.tag} "
                     f"and scripts/survey_objects.py {args.tag} first")

    layers, meta = load(args.tag, args.color, args.ground, args.all, args.halo)
    print(f"{sum(len(l[0]) for l in layers)} points"
          + (f", {len(meta.get('objects', []))} objects" if meta else "")
          + ("" if args.ground else f" (ground left out: --ground to show it)"))
    if args.export:
        export(args.tag, layers, args.up)
        return 0

    geoms = geometries(layers, meta)
    labels = object_labels(meta)
    if args.png:
        snapshot(args.png, geoms, args.point_size)
        return 0

    import open3d as o3d
    o3d.visualization.draw(geoms + [], title=f"{os.path.basename(args.tag)}: radar map (PX4 local NED)",
                           point_size=int(args.point_size), show_skybox=False, bg_color=(0.08, 0.08, 0.1, 1.0),
                           raw_mode=False, show_ui=True,
                           on_init=lambda vis: [vis.add_3d_label(at, text) for at, text in labels] +
                           [vis.add_3d_label(to_view(e), n) for e, n in (([5.5, 0, 0], "N"), ([0, 5.5, 0], "E"),
                                                                          ([0, 0, 5.5], "D"))])
    return 0


if __name__ == "__main__":
    sys.exit(main())
