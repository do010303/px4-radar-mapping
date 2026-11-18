"""Radar mounts of x500_radar3 and x500_radar6 (scripts/radar_layouts.py): no Gazebo needed."""

import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import radar_layouts as rl  # noqa: E402

MODELS = os.path.join(os.path.dirname(__file__), "..", "models")
C30, S30 = np.cos(np.radians(30)), np.sin(np.radians(30))
EXPECTED = {   # name: (boresight, direction of the 122 deg fan), base_link FLU
    "isk_up": ((0, 0, 1), (1, 0, 0)),
    "isk_fdown": ((C30, 0, -S30), (S30, 0, C30)),
    "isk_rdown": ((-C30, 0, -S30), (S30, 0, -C30)),
}


@pytest.mark.parametrize("name", sorted(EXPECTED))
def test_upright_ring_radars_look_where_meant(name):
    r = {n: r for n, r, _ in rl.mounts(6)}[name]
    boresight, fan = EXPECTED[name]
    assert np.allclose(r @ [1, 0, 0], boresight, atol=1e-9)
    # Up to its sign: the fan is symmetric.
    assert np.allclose(np.abs(r @ [0, 1, 0]), np.abs(fan), atol=1e-9)


def test_upright_ring_covers_every_elevation_ahead_and_behind():
    # Directions in the x-z plane all the way round: each in some upright radar's field of view.
    for a in np.radians(np.arange(0, 360, 2)):
        d = np.array([np.cos(a), 0.0, np.sin(a)])
        seen = False

        for name, r, _ in rl.mounts(6):
            local = r.T @ d
            az = np.degrees(np.arctan2(local[1], local[0]))
            el = np.degrees(np.arcsin(np.clip(local[2], -1, 1)))
            seen |= abs(az) <= 61 and abs(el) <= 20.5

        assert seen, f"{np.degrees(a):.0f} deg in the x-z plane is not covered"


def test_ring_radars_are_level_and_120_deg_apart():
    yaws = sorted(np.degrees(np.arctan2(r[1, 0], r[0, 0])) % 360 for _, r, _ in rl.mounts(3))
    assert np.allclose(yaws, [0, 120, 240], atol=1e-6)
    assert all(abs((r @ [1, 0, 0])[2]) < 1e-9 for _, r, _ in rl.mounts(3))


def test_coverage_of_the_layouts():
    assert rl.coverage(3) == pytest.approx(0.35, abs=0.02)
    assert rl.coverage(6) == pytest.approx(0.62, abs=0.02)


@pytest.mark.parametrize("layout", list(rl.LAYOUTS))
def test_vehicle_sdf_matches_the_table(layout):
    """models/<vehicle>/model.sdf is what make_isk_x3.sh wrote from the table: rerun it after editing."""
    root = ET.parse(os.path.join(MODELS, rl.VEHICLE[layout], "model.sdf")).getroot()
    poses = {inc.findtext("uri").removeprefix("model://radar_"): [float(v) for v in inc.findtext("pose").split()]
             for inc in root.iter("include") if inc.findtext("uri").startswith("model://radar_")}

    for name, xyz, rpy in rl.LAYOUTS[layout]:
        assert np.allclose(poses[name], [*xyz, *rpy], atol=1e-5)


def test_the_cube_candidate_reaches_the_most_six_isk_can_cover():
    bound = 6 * 2 * math.radians(61.0) * 2 * math.sin(math.radians(20.5)) / (4 * math.pi)
    assert rl.coverage("cube", n=60000) == pytest.approx(bound, abs=0.01)
    assert rl.off_boresight("cube", (0, 0, -1)) == pytest.approx(0.0, abs=1e-6)       # straight down on a boresight
    assert rl.off_boresight(6, (0, 0, -1)) > 55                                         # x500_radar6: fan edge


def test_x500_radar_aop_sees_nine_tenths_and_both_poles():
    assert rl.coverage("aop", n=60000) > 0.89
    assert rl.off_boresight("aop", (0, 0, -1)) == pytest.approx(0.0, abs=1e-6)
    assert rl.off_boresight("aop", (0, 0, 1)) == pytest.approx(0.0, abs=1e-6)
    assert rl.off_boresight("aop", (1, 0, 0)) == pytest.approx(0.0, abs=1e-6)     # the ISK leads
