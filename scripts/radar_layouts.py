"""Where the IWR6843ISK radars sit on the vehicles models/x500_radar3 and models/x500_radar6.

One table for both the vehicle SDFs (written by scripts/make_isk_x3.sh through this module) and
scripts/radar_array.py, so that the merged point cloud uses the mounts the simulation has.

Each radar's own frame: x is its boresight, its 122 deg (azimuth) fan spreads along y, its 41 deg
(elevation) one along z. Poses are base_link FLU, SDF convention: roll, pitch, yaw about fixed x, y, z.

  x500_radar3: a level ring under the body, yawed 0, +120, -120 deg: 360 deg around z, +-20 deg up and down.

  x500_radar6: that ring plus an upright one, 3 x 3: isk_up (straight up, on top), isk_fdown (ahead and
  30 deg down) and isk_rdown (behind and 30 deg down), rolled 90 deg so the 122 deg fan stands in the x-z
  plane: 360 deg around y (up, ahead, down, behind), +-20 deg sideways. The two rings cross at right
  angles; directions between them (up and to the left at 45 deg, say) are seen by neither: 62 % of all
  directions. (The authors of the paper put one radar on each face of a box, five of them 75 x 104 deg
  AOP: 82 %; see docs/paper_malle2026_notes.md.)

    python3 scripts/radar_layouts.py          # sky coverage of each layout
"""

import math
import os

import numpy as np

RING = [
    ("isk_0", (0.05, 0.0, -0.12), (0.0, 0.0, 0.0)),
    ("isk_120", (-0.025, 0.0433, -0.12), (0.0, 0.0, math.radians(120))),
    ("isk_240", (-0.025, -0.0433, -0.12), (0.0, 0.0, math.radians(240))),
]

VERTICAL = [
    # Rolled 90 deg: the 122 deg fan stands upright in the x-z plane; boresights 120 deg apart in it.
    # Above the top plate: the rotors, 25 cm out and level with it, stay below the 29 deg lower edge of its fan.
    ("isk_up", (0.0, 0.0, 0.10), (math.pi / 2, -math.pi / 2, 0.0)),
    ("isk_fdown", (0.07, 0.0, -0.15), (math.pi / 2, math.radians(30), 0.0)),
    ("isk_rdown", (-0.07, 0.0, -0.15), (math.pi / 2, math.radians(30), math.pi)),
]

# Not a vehicle yet: one radar on each face of a box, fans in a cycle (front and rear upright, left, right,
# top and bottom across). No two fields of view overlap, so it reaches the 71 % six of them can at most
# (6 x 1.49 sr of 4 pi); straight down and straight ahead under any tilt up to 60 deg are near a boresight.
# The authors' idea too: their forward ISK is rolled to an upright fan.
CUBE = [
    ("cube_front", (0.10, 0.0, -0.05), (math.pi / 2, 0.0, 0.0)),
    ("cube_rear", (-0.10, 0.0, -0.05), (math.pi / 2, 0.0, math.pi)),
    ("cube_left", (0.0, 0.10, -0.05), (0.0, 0.0, math.pi / 2)),
    ("cube_right", (0.0, -0.10, -0.05), (0.0, 0.0, -math.pi / 2)),
    ("cube_top", (0.0, 0.0, 0.10), (0.0, -math.pi / 2, 0.0)),
    ("cube_bottom", (0.0, 0.0, -0.15), (0.0, math.pi / 2, 0.0)),
]

# models/x500_radar_aop (RADARS=aop): the paper's mix, the ISK ahead (fan upright) and five IWR6843AOP (aop_*: 75 x 104 deg as
# measured in the paper, profiles/iwr6843aop.yaml; wires from about 7 m instead of 15). Rear: wide side across;
# left and right yawed 15 deg forward, wide side fore-aft; top and bottom wide side across. 91 % of all directions,
# 98 % of the band +-30 deg about the horizon, straight down and up on a boresight. What the authors' launch file
# means to mount; read with the TI radar frame it mounts something that sees 83 % (docs/paper_malle2026_notes.md).
# Rear and side AOP sit 15 cm below base_link: higher up, their 37.5 deg narrow half fan took in the rotors, 0.3-0.45 m
# from the centre (past the 0.3 m minimum range), and the safety sphere pushed the vehicle about (run #201).
ISK_AOP = [
    ("isk_front", (0.10, 0.0, -0.05), (math.pi / 2, 0.0, 0.0)),
    ("aop_rear", (-0.10, 0.0, -0.15), (math.pi / 2, 0.0, math.pi)),
    ("aop_left", (0.03, 0.10, -0.15), (math.pi / 2, 0.0, math.radians(75))),
    ("aop_right", (0.03, -0.10, -0.15), (-math.pi / 2, 0.0, -math.radians(75))),
    ("aop_top", (0.0, 0.0, 0.10), (-math.pi / 2, -math.pi / 2, 0.0)),
    ("aop_bottom", (0.0, 0.0, -0.15), (-math.pi / 2, math.pi / 2, 0.0)),
]

LAYOUTS = {3: RING, 6: RING + VERTICAL, "aop": ISK_AOP}
CANDIDATES = {"cube": CUBE}
# Half fields of view (azimuth along the radar's y, elevation along z), deg, by the prefix of the mount's name.
FOV = {"isk": (61.0, 20.5), "aop": (37.5, 52.0)}


def fov(name):
    return FOV["aop"] if name.startswith("aop_") else FOV["isk"]


def profile_path(name):
    """The radar profile a mount's model is built from: IWR6843AOP for aop_*, IWR6843ISK otherwise."""
    return os.path.join(PROFILES, "iwr6843aop.yaml" if name.startswith("aop_") else "iwr6843isk.yaml")


def layout_key(text):
    """--layout / RADARS value to a LAYOUTS key: "3" -> 3, "aop" -> "aop"."""
    return int(text) if str(text).isdigit() else text
VEHICLE = {3: "x500_radar3", 6: "x500_radar6", "aop": "x500_radar_aop"}
PROFILES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "profiles")


def rotation(rpy):
    """Radar-to-base_link rotation of an SDF roll, pitch, yaw: Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def mounts(layout):
    """[(name, rotation, position)] of a layout (a key of LAYOUTS or CANDIDATES), base_link frame."""
    table = LAYOUTS[layout] if layout in LAYOUTS else CANDIDATES[layout]
    return [(name, rotation(rpy), np.array(xyz)) for name, xyz, rpy in table]


def off_boresight(layout, direction):
    """Smallest angle (deg) between a base_link direction and the boresight of a radar that sees it; None if none does."""
    best = None

    for name, r, _ in mounts(layout):
        az_half, el_half = fov(name)
        local = r.T @ (np.asarray(direction, float) / np.linalg.norm(direction))

        if local[0] > 0 and abs(math.degrees(math.atan2(local[1], local[0]))) <= az_half \
                and abs(math.degrees(math.asin(np.clip(local[2], -1.0, 1.0)))) <= el_half:
            angle = math.degrees(math.acos(min(local[0], 1.0)))
            best = angle if best is None else min(best, angle)

    return best


def coverage(layout, n=30000, rng=None):
    """Share of all directions around the vehicle inside at least one radar's field of view."""
    rng = rng or np.random.default_rng(0)
    d = rng.normal(size=(n, 3))
    d /= np.linalg.norm(d, axis=1)[:, None]
    seen = np.zeros(n, dtype=bool)

    for name, r, _ in mounts(layout):
        az_half, el_half = fov(name)
        local = d @ r                       # each row: r.T @ direction
        az = np.degrees(np.arctan2(local[:, 1], local[:, 0]))
        el = np.degrees(np.arcsin(np.clip(local[:, 2], -1.0, 1.0)))
        seen |= (np.abs(az) <= az_half) & (np.abs(el) <= el_half)

    return float(seen.mean())


SDF = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <!-- Generated by scripts/make_isk_x3.sh from scripts/radar_layouts.py; edit the table there, not this file.
       {count} IWR6843 radars (profiles/iwr6843isk.yaml, iwr6843aop.yaml for aop_*), {coverage:.0f} % of all directions in view. -->
  <model name="{vehicle}">
    <include merge="true">
      <uri>model://x500</uri>
    </include>
{includes}
{joints}
  </model>
</sdf>
"""


def write_vehicle(layout, models_dir):
    """models/<vehicle>/model.sdf and model.config for a layout; the radar models (radar_<name>) must exist."""
    vehicle = VEHICLE[layout]
    includes, joints = [], []

    for name, xyz, rpy in LAYOUTS[layout]:
        includes.append(f"""    <include merge="true">
      <uri>model://radar_{name}</uri>
      <pose relative_to="base_link">{xyz[0]:g} {xyz[1]:g} {xyz[2]:g} {rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}</pose>
    </include>""")
        joints.append(f"""    <joint name="radar_{name}_joint" type="fixed">
      <parent>base_link</parent>
      <child>radar_link_{name}</child>
    </joint>""")

    os.makedirs(os.path.join(models_dir, vehicle), exist_ok=True)

    with open(os.path.join(models_dir, vehicle, "model.sdf"), "w") as f:
        f.write(SDF.format(vehicle=vehicle, count=len(LAYOUTS[layout]), coverage=100 * coverage(layout),
                           includes="\n".join(includes), joints="\n".join(joints)))

    with open(os.path.join(models_dir, vehicle, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{vehicle}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>x500 with {len(LAYOUTS[layout])} IWR6843 radars (scripts/radar_layouts.py)</description>
</model>
""")

    return vehicle


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:       # write the vehicle SDFs into this models directory
        for layout in LAYOUTS:
            print(f"{write_vehicle(layout, sys.argv[1])}: {len(LAYOUTS[layout])} radars")

    def show(v):
        return "blind" if v is None else f"{v:.0f} deg"

    for layout in [*LAYOUTS, *CANDIDATES]:
        name = VEHICLE.get(layout, f"{layout} (candidate)")
        ahead_tilted = show(off_boresight(layout, (math.cos(0.5), 0.0, math.sin(0.5))))
        print(f"{name}: {100 * coverage(layout):.0f} % of all directions in view; off boresight: straight down "
              f"{show(off_boresight(layout, (0, 0, -1)))}, level ahead at 29 deg nose down {ahead_tilted}")
