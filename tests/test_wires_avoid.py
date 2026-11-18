"""Wire detections and the reactive avoidance rules: no Gazebo needed."""

import argparse
import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import avoid_node as an  # noqa: E402
import radar_model as rm  # noqa: E402
from test_radar_model import ISK, config  # noqa: E402

with open(ISK) as f:
    SENSOR = yaml.safe_load(f)["sensor"]


def wire_at(range_m, radius, cfg, off_perpendicular_deg=0.0, runs=20):
    """Detection rate of a 35 m wire across the line of sight, range_m ahead, and its best SNR."""
    tilt = np.radians(off_perpendicular_deg)
    d = np.array([np.sin(tilt), np.cos(tilt), 0.0]) * 17.5
    centre = np.array([range_m, 0.0, 0.0])
    rng = np.random.default_rng(3)
    counts = [len(rm.wire_detections(centre - d, centre + d, radius, SENSOR, cfg, rng)) for _ in range(runs)]
    return float(np.mean(counts))


def test_a_20mm_wire_is_seen_across_the_line_of_sight_at_10_m():
    assert wire_at(10.0, 0.01, config()) > 10


def test_a_thinner_wire_is_seen_less_far():
    cfg = config()
    assert wire_at(10.0, 0.01, cfg) > wire_at(10.0, 0.005, cfg) > wire_at(10.0, 0.0006, cfg)
    assert wire_at(25.0, 0.0006, cfg) < 1       # the 1.2 mm steel wire is gone by 25 m (marginal to 15 m)
    assert wire_at(5.0, 0.0006, cfg) > 5        # and still there at 5 m


def test_a_wire_seen_obliquely_returns_less():
    # The same 2 m of wire 12 m ahead, square-on and turned 60 deg away from square-on.
    cfg = config(snr_sigma=0.0)
    centre = np.array([12.0, 0.0, 0.0])

    def best_snr(tilt_deg):
        t = np.radians(tilt_deg)
        d = np.array([np.sin(t), np.cos(t), 0.0])
        det = rm.wire_detections(centre - d, centre + d, 0.01, SENSOR, cfg, np.random.default_rng(1))
        return det[:, 4].max() if len(det) else -np.inf

    assert best_snr(0) > best_snr(60) + 5


def test_a_wire_outside_the_field_of_view_is_not_seen():
    cfg = config()
    p0, p1 = np.array([-10.0, -5.0, 0.0]), np.array([-10.0, 5.0, 0.0])        # behind the radar
    assert len(rm.wire_detections(p0, p1, 0.01, SENSOR, cfg, np.random.default_rng(1))) == 0


def test_wire_detections_lie_on_the_wire():
    cfg = config(sigma_az=0.0, sigma_el=0.0, sigma_range=0.0)
    p0, p1 = np.array([8.0, -10.0, 1.0]), np.array([8.0, 10.0, 1.0])
    det = rm.wire_detections(p0, p1, 0.01, SENSOR, cfg, np.random.default_rng(1))
    assert len(det) > 0
    assert np.allclose(det[:, 0], 8.0, atol=0.15)
    assert np.allclose(det[:, 2], 1.0, atol=0.15)


PARAMS = argparse.Namespace(r_a=6.0, r_s=1.5, k_t=1.0, k_s=1.5, a_max=3.0, margin=3.0, v_eb=0.3, v_max=5.0)
AHEAD = np.array([5.0, 0.0, 0.0])


def test_without_detections_the_pilot_is_not_changed():
    v, braking = an.avoid(AHEAD, AHEAD, np.empty((0, 3)), PARAMS, False)
    assert np.allclose(v, AHEAD) and not braking


def test_a_point_in_the_brake_cone_stops_the_vehicle():
    v, braking = an.avoid(AHEAD, AHEAD, np.array([[6.0, 0.5, 0.0]]), PARAMS, False)
    assert braking and np.allclose(v, 0.0)


def test_a_point_outside_the_cone_does_not_brake():
    v, braking = an.avoid(AHEAD, AHEAD, np.array([[3.0, 3.0, 0.0]]), PARAMS, False)
    assert not braking


def test_a_stopped_vehicle_stays_stopped_in_front_of_an_obstacle_it_would_run_into():
    # At rest, 5 m from a wire, with 5 m/s wanted: the stopping distance of that speed is 7 m.
    v, braking = an.avoid(AHEAD, np.zeros(3), np.array([[5.0, 0.0, 0.0]]), PARAMS, True)
    assert braking and np.allclose(v, 0.0)


def test_a_pole_to_the_right_pushes_the_velocity_to_the_left():
    # 28 deg off the heading: outside the brake cone, inside the avoidance sphere.
    v, braking = an.avoid(AHEAD, AHEAD, np.array([[4.0, -2.2, 0.0]]), PARAMS, False)
    assert not braking and v[1] > 0.3


def test_a_pole_to_the_left_pushes_the_velocity_to_the_right():
    v, _ = an.avoid(AHEAD, AHEAD, np.array([[4.0, 2.2, 0.0]]), PARAMS, False)
    assert v[1] < -0.3


def test_the_safety_sphere_pushes_away_from_a_close_point():
    v, _ = an.avoid(np.zeros(3), np.zeros(3), np.array([[0.0, 1.0, 0.0]]), PARAMS, False)
    assert v[1] < -0.3


def test_the_command_is_limited_to_the_top_speed():
    v, _ = an.avoid(np.array([9.0, 0.0, 0.0]), np.zeros(3), np.empty((0, 3)), PARAMS, False)
    assert np.linalg.norm(v) == pytest.approx(PARAMS.v_max)


def test_distance_to_a_wire_is_to_the_segment():
    wires = [("wire_a", np.array([0.0, -5.0, 10.0]), np.array([0.0, 5.0, 10.0]), 0.01)]
    assert an.distance_to_segments(np.array([-3.0, 0.0, 10.0]), wires)[0] == pytest.approx(2.99)
    assert an.distance_to_segments(np.array([0.0, 9.0, 10.0]), wires)[0] == pytest.approx(3.99)       # past the end


def test_the_brake_holds_until_below_the_brake_speed_even_if_the_cone_empties():
    v, braking = an.avoid(AHEAD, np.array([1.5, 0.0, 0.0]), np.empty((0, 3)), PARAMS, True)
    assert braking and np.allclose(v, 0.0)
    v, braking = an.avoid(AHEAD, np.array([0.1, 0.0, 0.0]), np.empty((0, 3)), PARAMS, True)
    assert not braking and np.allclose(v, AHEAD)


TANGENT = argparse.Namespace(r_a=8.0, r_s=1.5, k_t=1.0, k_s=1.5, a_max=3.0, margin=1.0, v_eb=2.0, v_max=5.0, tie=0.1, corridor=2.0, brake_reverse=0.0, way_round="shortest", switch=1.0, caution_radius=2.5, clearance=2.0)


def test_tangent_mode_climbs_over_a_wire_below_the_path():
    v, braking = an.tangent_avoid(AHEAD, AHEAD, np.array([[6.0, 0.0, -1.0]]), TANGENT, False)
    assert not braking and v[2] > 0.5
    assert np.linalg.norm(v) == pytest.approx(5.0)          # never faster than the wanted velocity


def test_tangent_mode_dips_under_a_wire_above_the_path():
    v, _ = an.tangent_avoid(AHEAD, AHEAD, np.array([[6.0, 0.0, 1.0]]), TANGENT, False)
    assert v[2] < -0.5


def test_tangent_mode_passes_over_a_level_wire():
    v, _ = an.tangent_avoid(AHEAD, AHEAD, np.array([[6.0, 0.0, 0.0]]), TANGENT, False)
    assert v[2] > 0.5


def test_tangent_mode_ignores_points_behind_and_beyond_r_a():
    pts = np.array([[-4.0, 0.0, -0.5], [9.0, 0.0, -0.5]])
    v, _ = an.tangent_avoid(AHEAD, AHEAD, pts, TANGENT, False)
    assert np.allclose(v, AHEAD)


def test_tangent_mode_brakes_only_above_the_brake_speed():
    wire = np.array([[4.0, 0.0, 0.0]])
    v, braking = an.tangent_avoid(AHEAD, AHEAD, wire, TANGENT, False)          # 5 m/s: l_H 5.2 m
    assert braking and np.allclose(v, 0.0)
    v, braking = an.tangent_avoid(AHEAD, np.array([1.5, 0.0, 0.0]), wire, TANGENT, True)
    assert not braking and v[2] > 0.5                                           # slow: tangent again


def test_tangent_mode_sends_only_the_push_in_the_safety_sphere():
    v, _ = an.tangent_avoid(AHEAD, np.zeros(3), np.array([[1.0, 0.0, 0.0]]), TANGENT, False)
    assert v[0] < 0 and abs(v[2]) < 1e-9


def test_tangent_mode_backs_off_while_braking_when_asked():
    p = argparse.Namespace(**{**vars(TANGENT), "brake_reverse": 2.0})
    v, braking = an.tangent_avoid(AHEAD, AHEAD, np.array([[4.0, 0.0, 0.0]]), p, False)
    assert braking and np.allclose(v, [-2.0, 0.0, 0.0])


def test_the_safety_sphere_does_not_release_the_brake():
    _, braking = an.tangent_avoid(AHEAD, AHEAD, np.array([[1.0, 0.0, 0.0]]), TANGENT, True)
    assert braking


def _line(start, end, n=25, jitter=0.03, seed=0):
    rng = np.random.default_rng(seed)
    return np.linspace(start, end, n) + rng.normal(scale=jitter, size=(n, 3))


def test_shortest_way_goes_round_a_pole_not_over_it():
    pole = _line([6.0, -0.6, -3.0], [6.0, -0.6, 3.0])               # upright, a little right of the path
    v, _ = an.tangent_avoid(AHEAD, AHEAD, pole, TANGENT, False)
    assert v[1] > 1.0 and abs(v[2]) < 0.5                            # left, level
    paper = argparse.Namespace(**{**vars(TANGENT), "way_round": "paper"})
    v, _ = an.tangent_avoid(AHEAD, AHEAD, pole, paper, False)
    assert abs(v[1]) < 0.5                                           # the paper's rule only goes up or down


def test_shortest_way_goes_left_round_a_pole_dead_ahead():
    v, _ = an.tangent_avoid(AHEAD, AHEAD, _line([6.0, 0.0, -3.0], [6.0, 0.0, 3.0]), TANGENT, False)
    assert v[1] > 1.0


def test_shortest_way_still_goes_over_a_wire_below():
    wire = _line([6.0, -3.0, -0.8], [6.0, 3.0, -0.8])
    v, _ = an.tangent_avoid(AHEAD, AHEAD, wire, TANGENT, False)
    assert v[2] > 0.5 and abs(v[1]) < 0.5


def test_shortest_way_leaves_a_block_on_the_right_to_the_left():
    yy, zz = np.meshgrid(np.linspace(-3.0, -0.8, 8), np.linspace(-2.0, 2.0, 8))
    face = np.column_stack((np.full(yy.size, 6.0), yy.ravel(), zz.ravel()))   # the face of a block, right of the path
    v, _ = an.tangent_avoid(AHEAD, AHEAD, face, TANGENT, False)
    assert v[1] > 1.0 and abs(v[2]) < 1.0


def test_shortest_way_takes_one_way_out_of_a_gap_between_wires():
    below = _line([6.5, -3.0, -1.2], [6.5, 3.0, -1.2], seed=1)       # a phase wire
    above = _line([7.0, -3.0, 0.6], [7.0, 3.0, 0.6], seed=2)         # the earth wire, closer to the vehicle's height
    v, _ = an.tangent_avoid(AHEAD, AHEAD, np.vstack((below, above)), TANGENT, False)
    assert v[2] > 1.0                                                # over the earth wire, not stuck between


def test_shortest_way_flies_through_a_gate_between_two_poles():
    gate = np.vstack((_line([6.0, 2.2, -3.0], [6.0, 2.2, 3.0], seed=3), _line([6.0, -2.2, -3.0], [6.0, -2.2, 3.0], seed=4)))
    v, _ = an.tangent_avoid(AHEAD, AHEAD, gate, TANGENT, False)
    assert np.allclose(v, AHEAD, atol=0.3)                           # both clear by more than r_s: straight on


def test_shortest_way_keeps_to_the_middle_of_a_gate_seen_through_noise():
    rng = np.random.default_rng(5)
    poles = [np.column_stack((np.full(40, 6.0), np.full(40, y) + rng.normal(scale=0.5, size=40), rng.uniform(-3, 3, 40)))
             for y in (2.2, -2.2)]
    v, _ = an.tangent_avoid(AHEAD, AHEAD, np.vstack(poles), TANGENT, False)
    assert abs(v[1]) < 1.0 and v[0] > 3.0                            # no swerve towards either pole


def test_nothing_past_the_waypoint_is_in_the_way():
    # Run #601: a waypoint 1.7 m ahead and a pole 6 m ahead behind it; the vehicle edged from side to side.
    pole = _line([6.0, 0.0, -3.0], [6.0, 0.0, 3.0])
    near_goal = argparse.Namespace(**{**vars(TANGENT), "state": {"horizon": 1.7}})
    v, _ = an.tangent_avoid(AHEAD, AHEAD, pole, near_goal, False)
    assert np.allclose(v, AHEAD)
    far_goal = argparse.Namespace(**{**vars(TANGENT), "state": {"horizon": 20.0}})
    v, _ = an.tangent_avoid(AHEAD, AHEAD, pole, far_goal, False)
    assert v[1] > 1.0


def _fresh(**kw):
    return argparse.Namespace(**{**vars(TANGENT), "state": {}, **kw})


def test_way_round_does_not_grow_with_the_number_of_points():
    # Run #603: a pole's hundreds of points sent 4.9 m/s sideways for a 0.1 m shift.
    pole = _line([6.0, 0.0, -3.0], [6.0, 0.0, 3.0])
    v1, _ = an.tangent_avoid(AHEAD, AHEAD, pole, _fresh(), False)
    v10, _ = an.tangent_avoid(AHEAD, AHEAD, np.vstack([pole] * 10), _fresh(), False)
    assert np.allclose(v1, v10)


def test_way_round_is_as_large_as_the_shift_missing():
    # A pole 1.5 m left of the path at 6 m: 0.5 m short of the clearance, a gentle step right; dead ahead,
    # 2 m to go, a steeper one; level, and the speed kept.
    beside, _ = an.tangent_avoid(AHEAD, AHEAD, _line([6.0, 1.5, -3.0], [6.0, 1.5, 3.0]), _fresh(), False)
    ahead, _ = an.tangent_avoid(AHEAD, AHEAD, _line([6.0, 0.0, -3.0], [6.0, 0.0, 3.0]), _fresh(), False)
    assert -1.5 < beside[1] < -0.2 and abs(ahead[1]) > abs(beside[1]) + 0.5
    assert abs(beside[2]) < 0.2 and abs(ahead[2]) < 0.2
    assert np.linalg.norm(ahead) == pytest.approx(np.linalg.norm(AHEAD), rel=1e-3)


def test_way_round_keeps_the_clearance_where_there_is_room():
    # A pole 1.8 m to the right is outside r_s (1.5) but inside the 2 m clearance; with clearance = r_s
    # nothing is in the way.
    pole = _line([6.0, -1.8, -3.0], [6.0, -1.8, 3.0], jitter=0.0)
    v, _ = an.tangent_avoid(AHEAD, AHEAD, pole, _fresh(), False)
    assert v[1] > 0.05
    v, _ = an.tangent_avoid(AHEAD, AHEAD, pole, _fresh(clearance=1.5), False)
    assert np.allclose(v, AHEAD)


def test_way_round_under_is_no_steeper_than_px4_descends():
    wire = _line([4.0, -3.0, 0.6], [4.0, 3.0, 0.6])              # just above the path, close: under it, steeply
    v, _ = an.tangent_avoid(AHEAD, np.array([1.9, 0.0, 0.0]), wire, _fresh(), False)   # slow: no brake
    assert v[2] == pytest.approx(-an.DOWN_MAX, abs=1e-6) and v[0] > 0.0


def test_one_noisy_point_does_not_close_a_gate():
    # Run #702: a return 1.3 m from the middle of a 4 m gate sent the vehicle round the outside of a pole.
    gate = np.vstack([_line([4.0, 2.0, -3.0], [4.0, 2.0, 3.0]), _line([6.0, -2.0, -3.0], [6.0, -2.0, 3.0])])
    v, _ = an.tangent_avoid(AHEAD, AHEAD, np.vstack([gate, [[4.0, 1.3, 0.0]]]), _fresh(), False)
    assert v[0] > 2.5 and abs(v[1]) < 2.5


def test_a_steep_way_round_slows_down_instead():
    pole = _line([3.5, 0.0, -3.0], [3.5, 0.0, 3.0])                # 2 m to go round it, 2 m to do it in
    v, _ = an.tangent_avoid(AHEAD, np.array([1.9, 0.0, 0.0]), pole, _fresh(), False)
    assert abs(v[1]) == pytest.approx(an.ASIDE_MAX, rel=1e-3) and v[0] < 2.5
