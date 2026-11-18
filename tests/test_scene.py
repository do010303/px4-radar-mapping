"""World obstacles and detection labelling: no Gazebo needed."""

import os
import sys

import numpy as np

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import scene  # noqa: E402

HOVER = os.path.join(ROOT, "worlds", "radar_hover.sdf")
BENCH = os.path.join(ROOT, "worlds", "radar_test.sdf")


def test_hover_world_obstacles_and_limits():
    obstacles = {o.name: o for o in scene.load_obstacles(HOVER)}
    assert "ground_plane" not in obstacles
    assert obstacles["wall"].size == (0.2, 4.0, 5.0)
    assert obstacles["pole"].size == (0.1, 0.1, 5.0)        # cylinder -> bounding box
    assert obstacles["wall"].min_rate == 0.95
    assert all(o.min_rate is not None for o in obstacles.values())


def test_bench_world_has_no_limits_and_finds_the_stand():
    obstacles = scene.load_obstacles(BENCH)
    assert {o.name for o in obstacles} == {"wall", "pole", "low_box", "overhead_bar"}
    assert all(o.min_rate is None for o in obstacles)
    assert np.allclose(scene.model_position(BENCH, "radar_stand"), (0, 0, 1.5))


def test_surface_distance_of_rotated_box():
    box = scene.Obstacle("b", (0.0, 0.0, 0.0), (2.0, 1.0, 1.0), np.pi / 2, None)
    # Rotated 90 deg: its 2 m side now runs along y.
    assert box.surface_distance(np.array([0.0, 0.9, 0.0]))[0] == 0.0
    assert np.isclose(box.surface_distance(np.array([1.0, 0.0, 0.0]))[0], 0.5)


def test_fit_ground_on_a_slope():
    rng = np.random.default_rng(0)
    xy = rng.uniform(-10, 10, (200, 2))
    z = 0.1 * xy[:, 0] + 0.5 + rng.normal(0, 0.03, 200)        # 5.7 deg slope, raised by 0.5 m
    clutter = rng.uniform((-10, -10, 2), (10, 10, 4), (20, 3))
    n, d = scene.fit_ground(np.vstack((np.column_stack((xy, z)), clutter)), np.array([0.0, 0.0, 3.0]))
    assert n[2] > 0.99
    assert abs(-(d + n[0] * 5.0) / n[2] - 1.0) < 0.1          # height at x = 5 is 1.0 m


def test_classify_prefers_obstacles_then_ground():
    wall = scene.Obstacle("wall", (8.0, 0.0, 2.5), (0.2, 4.0, 5.0), 0.0, 0.9)
    rng = np.random.default_rng(1)
    ground_pts = np.column_stack((rng.uniform(1, 6, (30, 2)), np.zeros(30)))
    pts = np.vstack((ground_pts, [[7.9, 0.0, 0.1], [7.9, 0.5, 3.0], [3.0, -3.0, 2.0]]))
    labels = scene.classify(pts, np.array([0.0, 0.0, 2.5]), [wall], sigma_el_deg=4.0)
    assert labels.on["wall"][-3] and labels.on["wall"][-2]    # the base of the wall is the wall
    assert labels.ground[:30].all()
    assert labels.unexplained[-1] and labels.unexplained.sum() == 1


def test_ground_in_front_of_a_far_obstacle_is_not_on_it():
    # At 575 m the lateral match distance is ~40 m, but the range must match too.
    tower = scene.Obstacle("tower", (575.0, 0.0, 15.0), (5.0, 5.0, 30.0), 0.0, 0.8)
    origin = np.array([0.0, 0.0, 2.6])
    pts = np.array([[545.0, 0.0, 0.0],      # ground 30 m short of the tower
                    [573.0, 8.0, 12.0]])    # on its face, 8 m off sideways (angle error)
    labels = scene.classify(pts, origin, [tower], sigma_el_deg=1.5)
    assert not labels.on["tower"][0]
    assert labels.on["tower"][1]


def test_true_origin_decides_what_a_detection_is_on():
    # A ground return at the foot of a far tower, displaced onto the tower by the
    # radar's elevation error, is not the tower; a return from the tower is.
    tower = scene.Obstacle("tower", (575.0, 0.0, 15.0), (5.0, 5.0, 30.0), 0.0, 0.3)
    origin = np.array([0.0, 0.0, 2.6])
    noisy = np.array([[572.0, 0.0, 10.0], [572.0, 0.0, 14.0]])
    truth = np.array([[565.0, 1.0, 0.0],      # ground, 7.5 m short of the tower
                      [572.5, 0.5, 12.0]])    # the tower's face
    labels = scene.classify(noisy, origin, [tower], sigma_el_deg=1.5, truth=truth)
    assert labels.on["tower"].tolist() == [False, True]
    # Without the true origin both look like the tower.
    assert scene.classify(noisy, origin, [tower], sigma_el_deg=1.5).on["tower"].all()


def test_in_view_of_forward_and_omni_radar():
    forward = {"az_min_deg": -60, "az_max_deg": 60, "el_min_deg": -40, "el_max_deg": 40, "range_max": 1000}
    omni = {"az_min_deg": -180, "az_max_deg": 180, "el_min_deg": -30, "el_max_deg": 30, "range_max": 50}
    ahead = scene.Obstacle("ahead", (10, 0, 2.5), (0.2, 4, 5), 0.0, None)
    behind = scene.Obstacle("behind", (-10, 0, 2.5), (0.2, 4, 5), 0.0, None)
    # Wide enough that its edge reaches into the 60 deg sector.
    beside = scene.Obstacle("beside", (0, 10, 2.5), (40, 0.2, 5), 0.0, None)
    far = scene.Obstacle("far", (80, 0, 2.5), (0.2, 4, 5), 0.0, None)
    position, level = np.array([0.0, 0.0, 2.5]), np.eye(3)
    assert scene.in_view(ahead, position, level, forward)
    assert not scene.in_view(behind, position, level, forward)
    assert scene.in_view(beside, position, level, forward)
    assert scene.in_view(behind, position, level, omni)
    assert not scene.in_view(far, position, level, omni)
    # Yawed 180 deg, the forward radar sees what was behind it.
    yawed = np.diag([-1.0, -1.0, 1.0])
    assert scene.in_view(behind, position, yawed, forward)
    assert not scene.in_view(ahead, position, yawed, forward)


def test_flight_worlds_have_a_gps_origin():
    # The flight worlds (those with hover test targets); without a GPS origin
    # PX4 gets no fix, so no position and no takeoff.
    import glob
    import xml.etree.ElementTree as ET

    for targets in glob.glob(os.path.join(ROOT, "worlds", "*.targets.yaml")):
        path = targets.replace(".targets.yaml", ".sdf")
        assert ET.parse(path).getroot().find("world/spherical_coordinates/latitude_deg") is not None, path
