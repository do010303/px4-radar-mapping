"""Radar model on synthetic ray grids: no Gazebo needed.

    cd examples/px4-mapping && python3 -m pytest tests
"""

import argparse
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import radar_model as rm  # noqa: E402


# The tests are calibrated for this profile, whichever one models/radar_mmwave
# was last built from. They also use it with antenna="omni" on a 360 deg grid.
ISK = os.path.join(os.path.dirname(__file__), "..", "profiles", "iwr6843isk.yaml")


def config(profile=ISK, **overrides):
    p = argparse.ArgumentParser()
    rm.add_model_args(p, profile)
    cfg = p.parse_args([])

    for k, v in overrides.items():
        setattr(cfg, k, v)

    return cfg


def grid(az_deg=(-180, 179, 1), el_deg=(-30, 30, 2)):
    az = np.radians(np.arange(az_deg[0], az_deg[1] + 1e-9, az_deg[2]))
    el = np.radians(np.arange(el_deg[0], el_deg[1] + 1e-9, el_deg[2]))
    return az, el


def directions(az, el):
    a, e = np.meshgrid(az, el)
    return np.stack((np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)), axis=-1)


def cast(az, el, planes, max_range=50.0):
    """Ranges[el, az] to the nearest of the given planes (normal, offset, bounds)."""
    d = directions(az, el)
    ranges = np.full(d.shape[:2], np.nan)

    for normal, offset, inside in planes:
        with np.errstate(divide="ignore", invalid="ignore"):
            t = -offset / (d @ np.asarray(normal))
            hit_pts = t[..., None] * d
        ok = (t > 0) & (t <= max_range) & inside(hit_pts)
        ranges = np.where(ok & ~(ranges < t), t, ranges)

    return ranges


def wall(x, half_width=2.0, z_lo=-1.5, z_hi=3.5):
    """Plane x = const, bounded in y and z (sensor frame)."""
    return ((1.0, 0.0, 0.0), -x,
            lambda p: (np.abs(p[..., 1]) <= half_width) & (p[..., 2] >= z_lo) & (p[..., 2] <= z_hi))


def ground(height):
    return ((0.0, 0.0, 1.0), height, lambda p: np.ones(p.shape[:-1], dtype=bool))


def test_wall_ahead_is_detected_at_its_range():
    az, el = grid()
    det = rm.detect(cast(az, el, [wall(10.0)]), az, el, config(), np.random.default_rng(0))
    ahead = det[np.abs(np.arctan2(det[:, 1], det[:, 0])) < np.radians(20)]
    assert len(ahead) > 0
    # On the plane x = 10 (slant ranges reach 10.8 m towards its corners; 15 deg cells and their angle
    # error spread it a little more).
    assert np.all(np.abs(ahead[:, 0] - 10.0) < 0.6)


def test_empty_scene_gives_no_detections():
    az, el = grid()
    det = rm.detect(np.full((len(el), len(az)), np.nan), az, el, config(), np.random.default_rng(0))
    assert det.shape == (0, 5)


def test_large_wall_and_thin_pole_reach():
    """The ISK profile: an 8 x 8 m wall at 15 m and a 10 cm pole at 10 m are seen, 30 m is out of reach.

    (At 20 m the wall is marginal: its 15 deg cells spread over several 0.122 m range bins.)"""
    az, el = grid((-61, 61, 1), (-20.5, 20.5, 1))
    big = lambda x: cast(az, el, [wall(x, half_width=4, z_lo=-2.6, z_hi=5.4)], max_range=50.0)
    seen = [len(rm.detect(big(15.0), az, el, config(), np.random.default_rng(k))) > 0 for k in range(20)]
    assert np.mean(seen) >= 0.8
    assert len(rm.detect(big(30.0), az, el, config(snr_sigma=0.0), np.random.default_rng(0))) == 0

    # A pole is a thin cylinder (scene.load_wires with poles), not a patch of a resolution cell.
    sensor = rm.load_profile(ISK)["sensor"]
    seen = [len(rm.wire_detections((10.0, 0.0, -2.6), (10.0, 0.0, 2.4), 0.05, sensor, config(),
                                   np.random.default_rng(k))) > 0 for k in range(20)]
    assert np.mean(seen) >= 0.8


def test_min_range_drops_close_returns():
    az, el = grid()
    # Every point of this patch is within 0.49 m of the sensor.
    det = rm.detect(cast(az, el, [wall(0.4, half_width=0.2, z_lo=-0.2, z_hi=0.2)]), az, el,
                    config(min_range=0.5), np.random.default_rng(0))
    assert len(det) == 0


def test_per_range_bin_and_max_points_caps():
    az, el = grid()
    ranges = cast(az, el, [ground(-2.5), wall(8.0, half_width=20)])
    cfg = config(sigma_range=0.0)
    det = rm.detect(ranges, az, el, cfg, np.random.default_rng(0))
    assert len(det) <= cfg.max_points
    # Before range noise every detection sits in its own bin; count per bin.
    _, counts = np.unique(np.floor(np.linalg.norm(det[:, :3], axis=1) / cfg.range_bin), return_counts=True)
    assert counts.max() <= cfg.per_range_bin + 1   # angle noise can move a point across a bin edge

    capped = rm.detect(ranges, az, el, config(max_points=10), np.random.default_rng(0))
    assert len(capped) <= 10


def test_radial_velocity_sign_and_size():
    """Moving towards a wall at 2 m/s: points ahead close at about -2 m/s, behind recede."""
    az, el = grid()
    ranges = cast(az, el, [wall(10.0), ((-1.0, 0.0, 0.0), -10.0,
                                        lambda p: (np.abs(p[..., 1]) <= 2) & (np.abs(p[..., 2]) <= 3))])
    det = rm.detect(ranges, az, el, config(antenna="omni"), np.random.default_rng(0), velocity=(2.0, 0.0, 0.0))
    ahead, behind = det[det[:, 0] > 5], det[det[:, 0] < -5]
    assert len(ahead) and len(behind)
    assert np.all(ahead[:, 3] < -1.5) and np.all(behind[:, 3] > 1.5)


def test_sector_antenna_loses_the_rear():
    az, el = grid()
    ranges = cast(az, el, [((-1.0, 0.0, 0.0), -10.0,
                            lambda p: (np.abs(p[..., 1]) <= 2) & (np.abs(p[..., 2]) <= 3))])
    omni = rm.detect(ranges, az, el, config(snr_sigma=0.0, antenna="omni"), np.random.default_rng(0))
    sector = rm.detect(ranges, az, el, config(snr_sigma=0.0, antenna="sector"), np.random.default_rng(0))
    assert len(omni) > 0 and len(sector) == 0


@pytest.mark.parametrize("az_span,expect_rings", [((-np.pi, np.pi - np.radians(1)), True),
                                                  ((-np.radians(60), np.radians(60)), False)])
def test_fov_segments(az_span, expect_rings):
    segs = rm.fov_segments(4.0, *az_span, -np.radians(30), np.radians(30))
    assert segs.shape[1:] == (2, 3)
    assert np.allclose(np.linalg.norm(segs[:, 1], axis=1)[segs[:, 1].any(axis=1)], 4.0)
    starts_at_sensor = np.all(segs[:, 0] == 0, axis=1).any()
    assert starts_at_sensor != expect_rings     # a sector draws rays from the sensor, 360 deg does not


def test_profile_defaults_match_the_built_model():
    """The tools' defaults come from models/radar_mmwave/profile.yaml."""
    profile = rm.load_profile()
    p = argparse.ArgumentParser()
    rm.add_model_args(p)
    cfg = p.parse_args([])

    for key, value in profile["model"].items():
        assert getattr(cfg, key) == value


def test_truth_is_the_noise_free_origin_of_each_detection():
    az, el = grid()
    det, truth = rm.detect(cast(az, el, [wall(10.0)]), az, el, config(sigma_az=5.0), np.random.default_rng(0),
                           truth=True)
    assert len(truth) == len(det) > 0
    assert np.allclose(truth[:, 0], 10.0, atol=1e-6)      # on the wall, whatever the angle error
    assert np.abs(det[:, 1] - truth[:, 1]).max() > 0.1     # the detections themselves are off


US_D1 = os.path.join(os.path.dirname(__file__), "..", "profiles", "us_d1.yaml")


def strip(x, y_lo, y_hi):
    """Plane x = const between two y values (sensor frame), 6 m tall."""
    return ((1.0, 0.0, 0.0), -x,
            lambda p: (p[..., 1] >= y_lo) & (p[..., 1] <= y_hi) & (np.abs(p[..., 2]) <= 3.0))


def detection_rate(cfg, az, el, planes, max_range=150.0, runs=30):
    ranges = cast(az, el, planes, max_range=max_range)
    return np.mean([len(rm.detect(ranges, az, el, cfg, np.random.default_rng(k))) > 0 for k in range(runs)])


def test_beam_pattern_is_3_db_down_at_half_the_beamwidth_one_way():
    cfg = config(US_D1)
    on = rm.two_way_beam_db(np.array(0.0), np.array(0.0), cfg)
    edge = rm.two_way_beam_db(np.radians(cfg.beam_az / 2), np.array(0.0), cfg)
    edge_el = rm.two_way_beam_db(np.array(0.0), np.radians(cfg.beam_el / 2), cfg)
    far = rm.two_way_beam_db(np.radians(80.0), np.radians(50.0), cfg)
    assert on == 0.0
    assert edge == pytest.approx(-6.0, abs=0.05) and edge_el == pytest.approx(-6.0, abs=0.05)
    assert far == pytest.approx(-2 * cfg.sidelobe_db)


def test_range_output_reports_one_range_on_boresight():
    az, el = grid((-43, 43, 1), (-30, 30, 1))
    ranges = cast(az, el, [wall(30.0, half_width=40.0, z_lo=-20, z_hi=20)], max_range=50.0)
    det = rm.detect(ranges, az, el, config(US_D1, output="range"), np.random.default_rng(0))
    assert det.shape == (1, 5)
    assert np.allclose(det[0, 1:3], 0.0)                 # no angle information
    assert abs(det[0, 0] - 30.0) < 1.0                   # the nearest return, the wall straight ahead


def test_beam_antenna_sees_on_boresight_but_not_outside_the_beam():
    az, el = grid((-43, 43, 1), (-30, 30, 1))
    cfg = config(US_D1, output="points", reflectivity_db=-30.0)      # a weak scatterer, so the beam shows
    assert detection_rate(cfg, az, el, [strip(35.0, -3, 3)], max_range=50.0) >= 0.8
    tan = np.tan(np.radians(40))
    assert detection_rate(cfg, az, el, [strip(35.0, 35 * tan - 3, 35 * tan + 3)], max_range=50.0) < 0.2
    tan = np.tan(np.radians(21))                          # the beam edge (-3 dB) is still seen
    assert detection_rate(cfg, az, el, [strip(35.0, 35 * tan - 3, 35 * tan + 3)], max_range=50.0) >= 0.8


def test_altimeter_reports_the_strongest_return_with_its_snr():
    az, el = grid((-43, 43, 1), (-30, 30, 1))
    ranges = cast(az, el, [wall(20.0, half_width=40.0, z_lo=-20, z_hi=20)], max_range=50.0)
    det = rm.detect(ranges, az, el, config(US_D1, snr_sigma=0.0), np.random.default_rng(0))
    assert det.shape == (1, 5)
    assert np.allclose(det[0, 1:3], 0.0)          # no angle
    assert abs(det[0, 0] - 20.0) < 0.5
    assert det[0, 4] > 13.0                       # a wall at 20 m is above the manual's 13 dB


def test_altimeter_reports_its_snr_even_when_below_the_threshold():
    az, el = grid((-43, 43, 1), (-30, 30, 1))
    small = cast(az, el, [strip(45.0, -0.4, 0.4)], max_range=50.0)
    cfg = config(US_D1, snr_sigma=0.0, reflectivity_db=-35.0)      # a poorly reflecting surface
    det = rm.detect(small, az, el, cfg, np.random.default_rng(0))
    assert len(det) == 1 and det[0, 4] < cfg.snr_threshold      # reported, but flagged by its SNR
    assert len(rm.detect(small, az, el, config(US_D1, snr_sigma=0.0, reflectivity_db=-35.0, output="points"),
                                 np.random.default_rng(0))) == 0


def test_altimeter_with_nothing_in_the_beam_reports_nothing():
    az, el = grid((-43, 43, 1), (-30, 30, 1))
    det = rm.detect(np.full((len(el), len(az)), np.nan), az, el, config(US_D1), np.random.default_rng(0))
    assert det.shape == (0, 5)


def test_altimeter_pick_nearest_ignores_a_stronger_far_return():
    az, el = grid((-43, 43, 1), (-30, 30, 1))
    near_weak = strip(12.0, 6.0, 7.5)       # off boresight, small
    far_strong = strip(35.0, -8.0, 8.0)
    ranges = cast(az, el, [near_weak, far_strong], max_range=50.0)
    nearest = rm.detect(ranges, az, el, config(US_D1, snr_sigma=0.0, pick="nearest"), np.random.default_rng(0))
    strongest = rm.detect(ranges, az, el, config(US_D1, snr_sigma=0.0, pick="strongest"), np.random.default_rng(0))
    assert len(nearest) == 1 and len(strongest) == 1
    assert nearest[0, 0] < strongest[0, 0]


def test_us_d1_packet_matches_the_manual():
    import us_d1_node as node

    # 20.00 m at 30 dB: altitude 2000 cm = 0x07D0.
    det = np.array([[20.0, 0.0, 0.0, 0.0, 30.0]])
    pkt = node.packet(det)
    assert pkt == bytes((0xFE, 0x02, 0xD0, 0x07, 30, (0x02 + 0xD0 + 0x07 + 30) & 0xFF))

    empty = node.packet(np.empty((0, 5)))
    assert empty[0:2] == bytes((0xFE, 0x02)) and empty[2] == 51 and empty[3] == 0   # ~0.51 m, manual 11


def test_us_d1_tty_carries_packets():
    import us_d1_node as node

    link = os.path.join(os.path.dirname(__file__), "..", "run", "test_us_d1_tty")
    master, slave = node.open_tty(link)
    try:
        os.write(master, node.packet(np.array([[5.0, 0.0, 0.0, 0.0, 25.0]])))
        with open(os.path.realpath(link), "rb", buffering=0) as f:
            assert f.read(6)[0] == 0xFE
    finally:
        os.close(master), os.close(slave), os.remove(link)
