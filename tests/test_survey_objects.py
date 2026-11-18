"""survey_objects: ground, poles and wires out of a synthetic radar map (no Gazebo, no flight)."""

import os
import sys
import warnings

import numpy as np

warnings.filterwarnings("ignore", message="A NumPy version")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import survey_objects as so  # noqa: E402


def _map(rng):
    """(N, 10) as survey_process.place gives it: x, y, z, v, snr, radar, frame, sensor x, y, z."""
    parts = [np.column_stack((rng.uniform(-5, 65, 4000), rng.uniform(-20, 20, 4000), rng.normal(0, 0.05, 4000)))]

    for x, y, h, n in ((15, 0, 13, 2500), (45, 0, 13, 2500), (35, -7, 8, 800)):
        # The radar's angle error spreads a pole's points by a metre or two.
        parts.append(np.column_stack((rng.normal(x, 0.7, n), rng.normal(y, 0.7, n), rng.uniform(0, h, n))))

    n = 1200
    parts.append(np.column_stack((rng.uniform(15, 45, n), rng.normal(3, 0.15, n), rng.normal(10, 0.15, n))))
    parts.append(np.column_stack((rng.uniform(-5, 65, 300), rng.uniform(-20, 20, 300), rng.uniform(1, 15, 300))))
    xyz = np.vstack(parts)
    frame = rng.integers(0, 3000, len(xyz))
    sensor = np.column_stack((rng.uniform(0, 60, len(xyz)), np.full(len(xyz), 14.0), np.full(len(xyz), 6.0)))
    return np.column_stack((xyz, np.zeros((len(xyz), 3)), frame, sensor))


def test_poles_and_wire_found():
    rng = np.random.default_rng(1)
    objects, ground_z = so.extract(_map(rng), np.random.default_rng(so.SEED), quiet=True)
    assert abs(ground_z) < 0.1
    poles = [o for o in objects if o["type"] == "pole"]
    wires = [o for o in objects if o["type"] == "wire"]

    for x, y in ((15, 0), (45, 0), (35, -7)):
        err = min(np.hypot(o["p"][0] - x, o["p"][1] - y) for o in poles)
        assert err < 0.3, (x, y, err)

    assert len(poles) == 3
    assert any(abs(np.mean([w["p0"][1], w["p1"][1]]) - 3) < 0.3 and abs(np.mean([w["p0"][2], w["p1"][2]]) - 10) < 0.3
               and abs(w["p1"][0] - w["p0"][0]) > 15 for w in wires)


def test_scattered_points_are_ghosts():
    rng = np.random.default_rng(2)
    xyz = rng.uniform([0, -20, 1], [60, 20, 15], (500, 3))
    pts = np.column_stack((xyz, np.zeros((500, 3)), np.arange(500), np.tile([30.0, 14.0, 6.0], (500, 1))))
    kept, n_ghost = so.remove_ghosts(pts)
    assert len(kept) == 0 and n_ghost == 500


def _pts(xyz):
    """xyz as find_poles / find_objects take them (columns beyond xyz unused there)."""
    return np.column_stack((xyz, np.zeros((len(xyz), 7))))


def test_pole_passed_on_one_side_is_a_pole_and_a_wall_is_not():
    rng = np.random.default_rng(3)
    n = 3000
    # Seen only from passes along x: stretched along x by the angle error (sd 0.9 by 0.3 m, ratio 3).
    pole = np.column_stack((rng.normal(20, 0.9, n), rng.normal(-10, 0.3, n), rng.uniform(0, 9, n)))
    # A 5 m wall along x.
    wall = np.column_stack((rng.uniform(37.5, 42.5, n), rng.normal(7, 0.15, n), rng.uniform(0, 6, n)))
    poles, _ = so.find_poles(_pts(np.vstack((pole, wall))), 0.0)
    assert len(poles) == 1
    assert np.hypot(poles[0]["p"][0] - 20, poles[0]["p"][1] + 10) < 0.3


def test_spread_beside_a_wire_is_not_an_object():
    rng = np.random.default_rng(4)
    wire = np.column_stack((rng.uniform(15, 45, 1500), rng.normal(0, 0.15, 1500), rng.normal(10, 0.15, 1500)))
    # A clump of the wire's spread, just beside it.
    clump = np.column_stack((rng.normal(36, 0.3, 60), rng.normal(0.5, 0.2, 60), rng.normal(10.3, 0.2, 60)))
    objects = so.find_objects(_pts(np.vstack((wire, clump))), 0.0, np.random.default_rng(so.SEED))
    assert [o["type"] for o in objects] == ["wire"]


def test_a_small_stack_on_a_wall_is_not_a_pole():
    rng = np.random.default_rng(5)
    # A wall's points, thin, plus a stack of 90 over one spot of it (as on live_6).
    wall = np.column_stack((rng.uniform(37.5, 42.5, 400), rng.normal(7, 0.15, 400), rng.uniform(2.5, 4.0, 400)))
    stack = np.column_stack((rng.normal(40.6, 0.15, 90), rng.normal(7.2, 0.1, 90), rng.uniform(2.5, 6.0, 90)))
    poles, _ = so.find_poles(_pts(np.vstack((wall, stack))), 0.0)
    assert poles == []


def test_sharpen_keeps_the_near_core_of_a_pole():
    rng = np.random.default_rng(6)
    # Seen from 3 m: close to the axis; from 25 m: spread a metre each way by the angle error.
    near = np.column_stack((rng.normal(20, 0.1, 1500), rng.normal(0, 0.1, 1500), rng.uniform(0, 9, 1500)))
    far = np.column_stack((rng.normal(20, 1.0, 3000), rng.normal(0, 1.0, 3000), rng.uniform(0, 9, 3000)))
    xyz = np.vstack((near, far))
    sensor = np.vstack((np.tile([17.0, 0.0, 5.0], (len(near), 1)), np.tile([20.0, -25.0, 5.0], (len(far), 1))))
    pts = np.column_stack((xyz, np.zeros((len(xyz), 4)), sensor))
    keep = so.sharpen(pts)
    pole = {"type": "pole", "p": [20.0, 0.0, 0.0], "h": 9.0}
    assert keep[:len(near)].mean() > 0.9
    assert so.object_spread(pole, pts[keep]) < 0.5 * so.object_spread(pole, pts)


def test_a_standing_wall_is_a_block_two_stacked_wires_are_not():
    rng = np.random.default_rng(7)
    # A 5 m x 0.4 m wall, 1 to 5.5 m up (aspect over BLOCK_MAX_ASPECT), and two wires 10 and 12 m up whose
    # spread joins them into one tall, long, thin cluster.
    wall = np.column_stack((rng.uniform(37.5, 42.5, 600), rng.normal(7, 0.1, 600), rng.uniform(1.0, 5.5, 600)))
    wires = np.vstack([np.column_stack((rng.uniform(15, 20, 400), rng.normal(0, 0.1, 400), rng.normal(z, 0.6, 400)))
                       for z in (10.0, 12.0)])
    blocks, _ = so.find_blocks(_pts(np.vstack((wall, wires))), 0.0)
    assert len(blocks) == 1 and abs(blocks[0]["c"][0] - 40) < 0.5
