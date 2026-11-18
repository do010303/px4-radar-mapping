#!/usr/bin/env python3
"""Write the radar's field-of-view wireframe as a mesh for models/radar_mmwave.

The field of view is read from the radar sensor in that model's model.sdf, so
rerun this after changing the sensor's angles.

The wireframe is a visual of the radar link, so Gazebo moves it with the
vehicle by itself. Each edge of radar_model.fov_segments becomes a thin square
tube. The radar sensor's visibility_mask excludes the visual, so the radar does
not detect its own wireframe.

    ./make_fov_mesh.py                  # wireframe at 4 m
    ./make_fov_mesh.py --range 6 --width 0.03
"""

import argparse
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

from radar_model import fov_segments

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(HERE, "..", "models", "radar_mmwave")


def sensor_angles(sdf_path):
    """(az_min, az_max, el_min, el_max) of the first lidar sensor in an SDF file."""
    scan = ET.parse(sdf_path).getroot().find(".//sensor/ray/scan")

    def limits(axis):
        node = scan.find(axis)
        return float(node.findtext("min_angle")), float(node.findtext("max_angle"))

    return limits("horizontal") + limits("vertical")


def tube_triangles(a, b, width):
    """Triangles of a square tube of the given width from a to b."""
    axis = b - a
    length = np.linalg.norm(axis)

    if length < 1e-9:
        return []

    axis /= length
    helper = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    h = width / 2.0
    offsets = [h * (u + v), h * (-u + v), h * (-u - v), h * (u - v)]
    ring_a = [a + o for o in offsets]
    ring_b = [b + o for o in offsets]
    tris = []

    for i in range(4):
        j = (i + 1) % 4
        tris += [(ring_a[i], ring_a[j], ring_b[j]), (ring_a[i], ring_b[j], ring_b[i])]

    tris += [(ring_a[0], ring_a[2], ring_a[1]), (ring_a[0], ring_a[3], ring_a[2])]
    tris += [(ring_b[0], ring_b[1], ring_b[2]), (ring_b[0], ring_b[2], ring_b[3])]
    return tris


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"radar field of view".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))

        for p, q, r in tris:
            n = np.cross(q - p, r - p)
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack("<12fH", *n, *p, *q, *r, 0))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--range", type=float, default=4.0, help="range at which the coverage is drawn (m)")
    p.add_argument("--width", type=float, default=0.02, help="line thickness (m)")
    p.add_argument("--sdf", default=os.path.join(MODEL_DIR, "model.sdf"), help="model with the radar sensor")
    p.add_argument("--out", default=os.path.join(MODEL_DIR, "meshes", "fov.stl"))
    args = p.parse_args()

    angles = sensor_angles(args.sdf)
    tris = [t for a, b in fov_segments(args.range, *angles) for t in tube_triangles(a, b, args.width)]
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    write_stl(args.out, tris)
    print(f"{len(tris)} triangles -> {os.path.normpath(args.out)}")


if __name__ == "__main__":
    main()
