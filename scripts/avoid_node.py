#!/usr/bin/env python3
"""Reactive obstacle avoidance on the radar point cloud, after Malle & Ebeid 2026 (arXiv:2602.03229).

A ROS 2 node (docs/paper_malle2026_notes.md): reads the merged detections of the radar ring
(/radar/points_map from scripts/radar_array.py: the detections in world axes, where they were detected), takes the velocity the pilot wants
(here: towards a goal) and sends PX4 the corrected velocity as an offboard setpoint (MAVLink,
SET_POSITION_TARGET_LOCAL_NED). No map, no tracking, no ground filter: only the detections of the
last cloud, placed relative to the vehicle's present position (it has moved since the detection).

Two rule sets (--mode). tangent (default, tangent_avoid): the paper's equations as written, the tangent
taken in the vertical plane, (p x z) x p, so a wire across the path is passed over or under and the
mission goes on (--waypoints). brake (avoid), this example's first version, three rules:

  avoidance sphere (r_a)  every point ahead of the wanted direction g inside r_a adds a tangent
                          t = (p x g) x p, p the unit vector to the point, to the velocity: slide around it;
  emergency brake         a point inside a cone around the direction of flight, half angle 0.245 rad,
                          as long as l_H = v^2 / (2 a_max) + margin: command zero velocity until the
                          speed is below v_eb, and stay stopped while the cone, sized for the wanted
                          speed, is not clear;
  safety sphere (r_s)     a point closer than r_s adds k_s (|p| - r_s) p / |p|: pushes away from it.

The paper gives no values for the radii, gains or a_max; the defaults below are this example's. The
authors' code (github.com/nhma20/spherical-radar-drone, offboard_control.cpp) uses others and adds rules
of its own: see docs/paper_malle2026_notes.md.

    source env.sh && ./scripts/avoid_node.py --goal 45 0 10         # needs run_sitl.sh RADARS=3 and radar_array.py
    ./scripts/avoid_node.py --waypoints 10,0,8 45,0,8 45,10,8      # a mission; the wires are at 10 m

Takes off, climbs to the goal's height, then flies at the goal; writes out/avoid.csv, and reports the
closest distance to the wires of the world (ground truth, scene.load_wires). Exit 0 when the vehicle
stayed at least --min-distance from every wire and obstacle, 1 otherwise, 2 for a setup problem.
"""

import argparse
import csv
import math
import os
import subprocess
import sys
import time

import numpy as np
from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from scene import load_obstacles, load_wires

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PX4_DIR = os.path.normpath(os.path.join(ROOT, "..", ".."))
_OWN = os.path.join(PX4_DIR, "build", "px4_sitl_mapping")
PX4_BIN = os.path.join(os.environ.get("PX4_BUILD") or (_OWN if os.path.exists(os.path.join(_OWN, "bin", "px4"))
                                                       else os.path.join(PX4_DIR, "build", "px4_sitl_default")), "bin")

CONE_HALF_ANGLE = 0.245     # rad, the paper's emergency-brake cone
BASELINE_MAX = 0.5          # s, the most radar delay taken as normal (avoid_node --max-lag)


class Params:
    def __init__(self, args):
        self.r_a, self.r_s = args.avoid_radius, args.safety_radius
        self.k_t, self.k_s = args.tangent_gain, args.safety_gain
        self.a_max, self.margin, self.v_eb = args.a_max, args.margin, args.brake_speed
        self.v_max = args.speed
        self.tie = getattr(args, "tie", 0.1)
        self.corridor = getattr(args, "corridor", 2.0)
        self.brake_reverse = getattr(args, "brake_reverse", 0.0)
        self.way_round = getattr(args, "way_round", "shortest")
        self.switch = getattr(args, "switch", 1.0)
        self.caution_radius = getattr(args, "caution_radius", 2.5)
        self.clearance = getattr(args, "clearance", 2.0)
        self.state = {}             # tangent_avoid's choices carried from one step to the next


def avoid(v_pilot, velocity, points, params, braking, debug=None):
    """The corrected velocity (ENU, m/s) and whether the emergency brake holds.

    v_pilot   velocity the pilot wants (3,)
    velocity  velocity of the vehicle (3,)
    points    detections relative to the vehicle in world-aligned axes (N, 3)
    braking   whether the emergency brake was on in the previous step
    debug     a dict to fill with the intermediate results, for display: heading, reach (cone length),
              in_cone, ahead and close (masks over points), tangent and push (velocity terms)
    """
    if debug is None:
        debug = {}

    debug.update(heading=None, reach=0.0, in_cone=None, ahead=None, close=None, tangent=np.zeros(3), push=np.zeros(3))
    want = np.linalg.norm(v_pilot)
    speed = np.linalg.norm(velocity)
    dist = np.linalg.norm(points, axis=1) if len(points) else np.empty(0)
    unit = points / np.maximum(dist, 1e-6)[:, None] if len(points) else points

    # The direction of flight, or the wanted one when (nearly) at rest.
    if speed > 0.5:
        heading = velocity / speed
    elif want > 1e-3:
        heading = v_pilot / want
    else:
        heading = None

    v_cmd = np.array(v_pilot, dtype=float)

    if heading is not None and len(points):
        # Sized for whichever is larger, the speed or the wanted speed: a vehicle that has come to
        # rest in front of an obstacle does not creep on towards it.
        reach = max(speed, want) ** 2 / (2.0 * params.a_max) + params.margin
        in_cone = (dist < reach) & (unit @ heading > math.cos(CONE_HALF_ANGLE))
        # Once on, the brake holds until the vehicle is below v_eb, whatever the next cloud shows.
        braking = bool(in_cone.any()) or (braking and speed > params.v_eb)
        debug.update(reach=reach, in_cone=in_cone)
    else:
        braking = braking and speed > params.v_eb

    debug["heading"] = heading

    if braking:
        v_cmd = np.zeros(3)
    elif want > 1e-3 and len(points):
        g = v_pilot / want
        ahead = (dist < params.r_a) & (unit @ g > 0.0)

        if ahead.any():
            p, d = unit[ahead], dist[ahead]
            t = g - (p @ g)[:, None] * p            # (p x g) x p
            w = 1.0 - d / params.r_a
            debug["tangent"] = params.k_t * want * (w[:, None] * t).sum(axis=0) / w.sum() * w.max()
            v_cmd = v_cmd + debug["tangent"]

        debug["ahead"] = ahead

    if len(points):
        close = dist < params.r_s
        debug["close"] = close

        if close.any():
            push = (params.k_s * (dist[close] - params.r_s))[:, None] * unit[close]
            debug["push"] = np.clip(push.sum(axis=0), -2.0, 2.0)
            v_cmd = v_cmd + debug["push"]

    norm = np.linalg.norm(v_cmd)

    if norm > params.v_max:
        v_cmd = v_cmd * params.v_max / norm

    return v_cmd, braking


# Candidate shifts of the line of flight, in the plane square to it: 16 directions, 0.1 m steps out to 5 m.
_ANGLES = np.radians(np.arange(0, 360, 22.5))
_STEPS = np.arange(0.1, 5.01, 0.1)
SHIFTS = np.concatenate([[[0.0, 0.0]], (np.stack((np.cos(_ANGLES), np.sin(_ANGLES)), axis=1)[None, :, :]
                                         * _STEPS[:, None, None]).reshape(-1, 2)])
# Near ties in a fixed order: up (as the paper climbs a level wire), then left, right, down (m added).
_BIAS = 0.05 * np.where(np.abs(np.arctan2(SHIFTS[:, 1], SHIFTS[:, 0]) - np.pi / 2) < 0.3, 0.0,
                        np.where(SHIFTS[:, 0] > 0.3, 1.0, np.where(SHIFTS[:, 0] < -0.3, 2.0, 3.0)))
_BIAS[0] = 0.0      # no shift is no direction: it was charged as down, and a 0.1 m step up beat it


CLEARANCE_WEIGHT = 2.0       # cost of each metre the line passes closer than params.clearance, in metres of shift
SAFETY_WEIGHT = 10.0         # and closer than r_s; only within r_s / 2 is a shift ruled out
ASIDE_MAX = 2.5              # m/s, the most the line is moved at; a steeper way round slows the vehicle instead
UP_MAX, DOWN_MAX = 3.0, 1.5  # m/s, PX4's MPC_Z_VEL_MAX_UP / _DN: a way round over or under no steeper than these


def way_round(points, v_pilot, params, state):
    """The smallest shift of the line of flight, square to it, that keeps every point ahead params.clearance
    away from it (r_s at least, the clearance where there is room): (the shift (3,) in metres, the distance
    ahead of the nearest point the present line passes too close to), or None when no shift clears r_s.

    A wire across the path is passed over or under, as in the paper; a pole beside, a block on one side by
    the other; the phase wires with the earth wire above them over both; between the two poles of a gate,
    straight on. The paper takes each point's own vertical tangent instead: those of a pole's lower points
    lean into it, and over and under cancel in a gap between wires (runs #421, #441). Kept, unless a shift
    the other way is shorter by params.switch metres.

    The clearance is a cost, not a bound: a gate narrower than twice it is still flown through the middle,
    not gone round as a whole. So, steeply, is r_s, down to r_s / 2: one noisy point is not a wall.
    """
    if not len(points):
        return np.zeros(3), np.inf

    flat = np.array([v_pilot[0], v_pilot[1], 0.0])
    ahead = flat / np.linalg.norm(flat) if np.linalg.norm(flat) > 1e-3 else np.array([1.0, 0.0, 0.0])
    left = np.cross([0.0, 0.0, 1.0], ahead)
    q = np.stack((points @ left, points[:, 2]), axis=1)                 # offsets square to the flight
    gap = np.sqrt(((q[None, :, :] - SHIFTS[:, None, :]) ** 2).sum(axis=2)).min(axis=1)
    clearance = max(params.clearance, params.r_s)
    cost = (np.linalg.norm(SHIFTS, axis=1) + _BIAS + CLEARANCE_WEIGHT * np.maximum(clearance - gap, 0.0)
            + SAFETY_WEIGHT * np.maximum(params.r_s - gap, 0.0))
    keep = state.get("shift")

    if keep is not None:
        # The other way round costs the switch margin more: no swapping sides on noise.
        cost = cost + params.switch * (SHIFTS @ keep < 0)

    # One noisy return 1.3 m from the middle of a gate had ruled the gate out and sent the vehicle round the
    # outside of a pole it was 2 m from (run #702): r_s is a steep cost, the bound is r_s / 2.
    cost[gap < 0.5 * params.r_s] = np.inf
    state["gap"] = float(gap[0])        # how close the present line passes, for the log

    if not np.isfinite(cost).any():
        return None

    i = int(cost.argmin())
    best = SHIFTS[i]
    size = np.linalg.norm(best)

    if size < 1e-6:
        return np.zeros(3), np.inf

    state["shift"], state["idle"] = best / size, 0
    # How soon the shift must be done: by the nearest point the present line passes closer than the shifted one.
    blocking = np.linalg.norm(q, axis=1) < gap[i]
    reach = float((points[blocking] @ ahead).min()) if blocking.any() else np.inf
    return best[0] * left + best[1] * np.array([0.0, 0.0, 1.0]), reach


def way_round_velocity(v_pilot, shift, reach, params):
    """The velocity along the tangent of way_round's shift: the wanted velocity with a part square to it that
    moves the line by the shift before the vehicle is within r_s of the nearest point in the way, at the
    slope shift / (reach - r_s), as the paper's tangent grazes the safety sphere of a point; the same law
    over, under, left and right. The slope, not the number of radar points, sets the part (run #603: a
    pole's hundreds of points sent 4.9 m/s sideways for a 0.1 m shift). At least min(shift, 1) m/s of it,
    so that a vehicle held up in front of an obstacle still edges round it.
    """
    size = np.linalg.norm(shift)
    want = np.linalg.norm(v_pilot)

    if size < 1e-6 or params.k_t <= 0.0:
        return v_pilot, np.zeros(3)

    forward = np.linalg.norm(v_pilot[:2])
    room = max(reach - params.r_s, 0.5)
    aside = params.k_t * max(forward * size / room, min(size, 1.0))
    ahead = v_pilot

    if aside > ASIDE_MAX:
        # Too steep to be flown at this speed: slower ahead instead of faster aside, the slope kept.
        ahead = v_pilot * ASIDE_MAX / aside
        aside = ASIDE_MAX

    side = aside * shift / size
    out = ahead + side
    # No faster than wanted (or than the edging speed), and no steeper up or down than PX4 flies: scaled
    # as a whole, the slope stays.
    scale = min(1.0, max(want, min(aside, 1.0)) / max(np.linalg.norm(out), 1e-6))

    if out[2] * scale > UP_MAX:
        scale = UP_MAX / out[2]
    elif out[2] * scale < -DOWN_MAX:
        scale = -DOWN_MAX / out[2]

    return out * scale, side * scale


def tangent_avoid(v_pilot, velocity, points, params, braking, debug=None):
    """The paper's rules as written (Sec. IV): the corrected velocity (ENU, m/s) and whether the
    emergency brake holds. Same arguments as avoid().

      tangent         every point p in r_a towards which the wanted velocity v_u (or the velocity v_d)
                      points adds (p.v) t, t = (p x z) x p: the tangent of p in the vertical plane
                      through it, with the sign that agrees with v. A wire across the path is passed
                      over or under, not braked for: below the vehicle t leans up, above it down. As in
                      the authors' code, each term is weighted by the depth in r_a,
                      1 - (|p| - r_s) / (r_a - r_s), and by k_t |v|; then
                      v_out = min(1, |v_u| / |t_s + v_u|) (t_s + v_u).
      emergency brake only above v_eb: a point in the cone along v_d, as long as l_H, commands zero
                      velocity until the speed is below v_eb; then the tangent rule again.
      safety sphere   a point closer than r_s: the push away from it alone is sent.

    Beyond the paper, with params.way_round "shortest" (the default; "paper" is the paper as written): the
    points in the way push along the smallest shift of the line of flight that clears everything ahead by r_s
    (way_round): over or under a wire, beside a pole, past a block by its free side, straight through a gate.
    The velocity follows the tangent that clears the nearest of them by the shift (way_round_velocity).

    Also beyond the paper: only points within params.corridor of the line along v count for the tangent.
    The paper's plane is the one across a horizontal wire; for an upright pole beside the path the
    vertical tangent of its lower points leans towards the pole, and it pulled the vehicle into it.

    A point dead level with the vehicle has no preferred side (t.v = 0): it counts as below (climb)
    while |t.v| < params.tie, so the noisy elevations of a level wire do not cancel out.
    """
    if debug is None:
        debug = {}

    debug.update(heading=None, reach=0.0, in_cone=None, ahead=None, close=None, tangent=np.zeros(3), push=np.zeros(3))
    want = np.linalg.norm(v_pilot)
    speed = np.linalg.norm(velocity)
    v_pilot = np.asarray(v_pilot, dtype=float)

    if not len(points):
        braking = braking and speed > params.v_eb
        debug["heading"] = velocity / speed if speed > 0.5 else None
        return (np.zeros(3) if braking else v_pilot), braking

    dist = np.linalg.norm(points, axis=1)
    unit = points / np.maximum(dist, 1e-6)[:, None]
    close = dist < params.r_s
    debug["close"] = close

    if close.any():
        push = (params.k_s * (dist[close] - params.r_s))[:, None] * unit[close]
        debug["push"] = np.clip(push.sum(axis=0), -2.0, 2.0)
        return debug["push"], braking and speed > params.v_eb

    if speed > 0.5:
        heading = velocity / speed
        debug["heading"] = heading

    if speed > params.v_eb:
        reach = speed ** 2 / (2.0 * params.a_max) + params.margin
        in_cone = (dist < reach) & (unit @ heading > math.cos(CONE_HALF_ANGLE))
        braking = braking or bool(in_cone.any())
        debug.update(reach=reach, in_cone=in_cone)
    else:
        braking = False

    if braking:
        # Zero is the paper's "maximum braking"; the authors' code backs off at 2 m/s along -v.
        return -heading * params.brake_reverse, True

    if params.caution_radius > params.r_s:
        # The authors' caution sphere (offboard_control.cpp, not in the paper): within r_c of a point, the part
        # of the wanted velocity towards it is taken away, all of it at r_s. A waypoint just past a wire then
        # no longer pulls the vehicle back up into the wire it is passing under.
        near = np.flatnonzero(dist < params.caution_radius)

        for i in near[np.argsort(dist[near])][:60]:
            into = v_pilot @ unit[i]

            if into > 0.0:
                share = min((params.caution_radius - dist[i]) / (params.caution_radius - params.r_s), 1.0)
                v_pilot = v_pilot - share * into * unit[i]

        debug["v_caution"] = v_pilot
        want = np.linalg.norm(v_pilot)

    # (p x z) x p = z - (p.z) p: the vertical tangent, sign still free.
    t = np.array([0.0, 0.0, 1.0]) - unit[:, 2:3] * unit
    t /= np.maximum(np.linalg.norm(t, axis=1), 1e-6)[:, None]
    depth = np.clip(1.0 - (dist - params.r_s) / (params.r_a - params.r_s), 0.0, 1.0)
    state = params.state if hasattr(params, "state") else {}
    # Nothing past the waypoint (by r_s) is in the way: the vehicle stops there. Without this, a pole beyond a
    # waypoint swung the line of flight from side to side as the vehicle edged round the waypoint (run #601).
    horizon = state.get("horizon", np.inf) + params.r_s
    t_sum = np.zeros(3)
    ahead = np.zeros(len(points), dtype=bool)
    masks = []

    for v, size in ((v_pilot, want), (velocity, speed)):
        if size < 1e-3:
            continue

        v_hat = v / size
        along = unit @ v_hat
        # Only what lies in the corridor along v: a pole beside the path is not in the way, and the
        # points of it below the vehicle would turn a vertical tangent towards it.
        lateral = dist * np.sqrt(np.clip(1.0 - along ** 2, 0.0, 1.0))
        mask = (dist < params.r_a) & (along > 0.0) & (lateral < params.corridor) & (dist * along < horizon)
        masks.append((v_hat, size, along, mask))
        ahead |= mask

    escape = None

    if params.way_round == "shortest" and ahead.any():
        # The shift is checked against everything ahead within 5 m of the line, not just the corridor: a
        # shift must not move the line onto an obstacle beside it.
        v_hat = masks[0][0]
        along = unit @ v_hat
        radial = dist * np.sqrt(np.clip(1.0 - along ** 2, 0.0, 1.0))
        around = (dist < params.r_a) & (along > 0.0) & (radial < 5.0) & (dist * along < horizon)
        escape = way_round(points[around], v_pilot, params, state)

    if escape is not None:
        out, side = way_round_velocity(v_pilot, *escape, params)
        debug.update(ahead=ahead, tangent=side, shift=escape[0])
        return out, False

    for v_hat, size, along, mask in masks:
        if not mask.any():
            continue

        agree = t[mask] @ v_hat
        sign = np.where(agree > params.tie, 1.0, np.where(agree < -params.tie, -1.0, np.sign(t[mask, 2] + 1e-9)))
        t_sum += params.k_t * size * ((along[mask] * depth[mask] * sign)[:, None] * t[mask]).sum(axis=0)

    if not ahead.any():
        # Forgotten after a second (20 steps) with nothing in the way, not at the first empty cloud.
        state["idle"] = state.get("idle", 0) + 1

        if state["idle"] > 20:
            state.pop("shift", None)

    debug.update(ahead=ahead, tangent=t_sum)
    out = v_pilot + t_sum
    norm = np.linalg.norm(out)
    return (out * min(1.0, want / norm) if norm > 1e-6 else np.zeros(3)), False


class LogicView:
    """The avoidance rules drawn in RViz: /avoid/markers (visualization_msgs/MarkerArray, frame map).

      blue sphere        avoidance sphere r_a: points ahead in it (magenta) bend the velocity (tangent rule)
      red sphere         safety sphere r_s: points in it (red, large) push the vehicle away
      cone               emergency-brake cone along the direction of flight, as long as l_H; orange, red when
                         braking; points inside it are drawn red
      arrows             green: wanted velocity (pilot), white: velocity of the vehicle, yellow: command sent
                         to PX4, magenta: tangent term, red: safety push
      text               the rule in force and the speeds
    """

    def __init__(self, node, params):
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker, MarkerArray

        self.Point, self.Marker, self.MarkerArray = Point, Marker, MarkerArray
        self.node, self.params = node, params
        self.pub = node.create_publisher(MarkerArray, "/avoid/markers", 1)

    def _marker(self, mid, kind, rgba, stamp):
        m = self.Marker()
        m.header.frame_id, m.header.stamp = "map", stamp
        m.ns, m.id, m.type, m.action = "avoid", mid, kind, self.Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = (float(c) for c in rgba)
        return m

    def _pt(self, p):
        return self.Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))

    def publish(self, pos, vel, v_pilot, v_cmd, braking, points, debug):
        stamp = self.node.get_clock().now().to_msg()
        out = self.MarkerArray()
        M = self.Marker

        for mid, radius, rgba in ((0, self.params.r_a, (0.2, 0.5, 1.0, 0.08)), (1, self.params.r_s, (1.0, 0.1, 0.1, 0.2))):
            m = self._marker(mid, M.SPHERE, rgba, stamp)
            m.pose.position = self._pt(pos)
            m.scale.x = m.scale.y = m.scale.z = 2.0 * radius
            out.markers.append(m)

        # Brake cone: rays from the vehicle to a circle at l_H around the heading.
        cone = self._marker(2, M.LINE_LIST, (1.0, 0.1, 0.1, 0.9) if braking else (1.0, 0.6, 0.1, 0.7), stamp)
        cone.scale.x = 0.04
        heading = debug.get("heading")

        if heading is not None and debug.get("reach", 0.0) > 0.0:
            h = heading / np.linalg.norm(heading)
            a = np.cross(h, [0.0, 0.0, 1.0]) if abs(h[2]) < 0.9 else np.cross(h, [1.0, 0.0, 0.0])
            a /= np.linalg.norm(a)
            b = np.cross(h, a)
            reach = debug["reach"]
            radius = reach * math.tan(CONE_HALF_ANGLE)
            ring = [pos + h * reach + radius * (math.cos(t) * a + math.sin(t) * b) for t in np.linspace(0, 2 * math.pi, 17)]

            for i in range(16):
                cone.points += [self._pt(ring[i]), self._pt(ring[i + 1])]

            for i in range(0, 16, 4):
                cone.points += [self._pt(pos), self._pt(ring[i])]

        out.markers.append(cone)

        # Points that drive a rule, in the colour of the rule.
        for mid, key, rgba, size in ((3, "ahead", (0.9, 0.2, 0.9, 0.9), 0.25), (4, "in_cone", (1.0, 0.1, 0.1, 1.0), 0.35),
                                     (5, "close", (1.0, 0.0, 0.0, 1.0), 0.5)):
            m = self._marker(mid, M.SPHERE_LIST, rgba, stamp)
            m.scale.x = m.scale.y = m.scale.z = size
            mask = debug.get(key)

            if mask is not None and len(points):
                m.points = [self._pt(p + pos) for p in points[mask]]

            out.markers.append(m)

        for mid, vec, rgba in ((6, v_pilot, (0.1, 0.9, 0.1, 1.0)), (7, vel, (1.0, 1.0, 1.0, 1.0)),
                               (8, v_cmd, (1.0, 0.9, 0.1, 1.0)), (9, debug.get("tangent", np.zeros(3)), (0.9, 0.2, 0.9, 1.0)),
                               (10, debug.get("push", np.zeros(3)), (1.0, 0.1, 0.1, 1.0))):
            m = self._marker(mid, M.ARROW, rgba, stamp)
            m.scale.x, m.scale.y, m.scale.z = 0.08, 0.18, 0.25
            m.points = [self._pt(pos), self._pt(pos + vec)]

            if np.linalg.norm(vec) < 1e-3:
                m.color.a = 0.0

            out.markers.append(m)

        if braking:
            rule = "EMERGENCY BRAKE"
        elif np.linalg.norm(debug.get("push", np.zeros(3))) > 1e-3:
            rule = "SAFETY SPHERE"
        elif np.linalg.norm(debug.get("tangent", np.zeros(3))) > 1e-3:
            t = debug["tangent"]
            side = np.cross([0.0, 0.0, 1.0], v_pilot)       # left of the wanted direction
            sideways = float(t @ side / max(np.linalg.norm(side), 1e-6))

            if abs(t[2]) >= abs(sideways):
                rule = "TANGENT " + ("(over)" if t[2] > 0 else "(under)")
            else:
                rule = "TANGENT " + ("(left)" if sideways > 0 else "(right)")
        else:
            rule = "free flight"

        text = self._marker(11, M.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0), stamp)
        text.pose.position = self._pt(pos + np.array([0.0, 0.0, 2.0]))
        text.scale.z = 0.6
        text.text = (f"{rule}\nwanted {np.linalg.norm(v_pilot):.1f}  cmd {np.linalg.norm(v_cmd):.1f}  "
                     f"speed {np.linalg.norm(vel):.1f} m/s\n{len(points)} radar points")
        out.markers.append(text)
        self.pub.publish(out)


def distance_to_segments(position, wires):
    """Distance from a point to the nearest of the wires (name, p0, p1, radius), minus the wire radius."""
    best = (math.inf, None)

    for name, p0, p1, radius in wires:
        d = p1 - p0
        s = np.clip(np.dot(position - p0, d) / np.dot(d, d), 0.0, 1.0)
        dist = np.linalg.norm(position - (p0 + s * d)) - radius

        if dist < best[0]:
            best = (dist, name)

    return best


def px4(*cmd):
    return subprocess.run([os.path.join(PX4_BIN, f"px4-{cmd[0]}"), *cmd[1:]], capture_output=True, text=True, timeout=20)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--world", default="power_line")
    p.add_argument("--goal", type=float, nargs=3, default=(45.0, 0.0, 10.0), metavar=("X", "Y", "Z"),
                   help="Gazebo world coordinates (m); the vehicle spawns at the origin")
    p.add_argument("--waypoints", nargs="+", default=None, metavar="X,Y,Z",
                   help="a mission instead of --goal: flown in turn, each reached within --accept")
    p.add_argument("--accept", type=float, default=1.0, help="waypoint acceptance radius (m)")
    p.add_argument("--mode", choices=("tangent", "brake"), default="tangent",
                   help="tangent: the paper's rules as written, vertical tangent (tangent_avoid); "
                        "brake: tangent towards the goal and a holding brake (avoid)")
    p.add_argument("--corridor", type=float, default=2.0,
                   help="tangent mode: points farther than this from the line of flight are not in the way (m)")
    p.add_argument("--proximity-test", action="store_true",
                   help="the goal lies on an obstacle (close-proximity test): PASS when never closer than "
                        "--min-distance, the mission need not be finished")
    p.add_argument("--cmd-accel", type=float, default=3.0,
                   help="largest rise of the commanded speed (m/s^2); turns, braking and the safety push go out "
                        "at once; 0: none")
    p.add_argument("--detect-range", type=float, default=0.0,
                   help="range at which the radars see the thinnest obstacle (m): caps the cruise speed at "
                        "sqrt(2 a_max (range - margin)); 0: no cap")
    p.add_argument("--brake-reverse", type=float, default=0.0,
                   help="tangent mode: speed commanded against the velocity while braking (m/s); 0 as in the "
                        "paper, 2 as in the authors' code")
    p.add_argument("--way-round", choices=("shortest", "paper"), default="shortest",
                   help="tangent mode: shortest: one way round all that is in the way, the shortest of over, under, "
                        "left and right (way_round); paper: each point's own (p x z) x p, over or under only")
    p.add_argument("--caution-radius", type=float, default=2.5,
                   help="tangent mode: r_c of the authors' caution sphere, the wanted velocity towards points within "
                        "it is cut, all of it at r_s (m); 0: off")
    p.add_argument("--clearance", type=float, default=2.0,
                   help="tangent mode, --way-round shortest: how far the line of flight is shifted clear of what is "
                        "in the way where there is room; r_s at least (m)")
    p.add_argument("--switch", type=float, default=1.0,
                   help="way round kept until another is shorter by this much (m)")
    p.add_argument("--tie", type=float, default=0.1,
                   help="tangent mode: |t.v| below which a point counts as level and is passed over")
    p.add_argument("--speed", type=float, default=5.0, help="cruise speed towards the goal (m/s)")
    p.add_argument("--climb-speed", type=float, default=2.0)
    p.add_argument("--avoid-radius", type=float, default=None, help="r_a (m); default 8 tangent, 6 brake")
    p.add_argument("--safety-radius", type=float, default=1.5, help="r_s (m)")
    p.add_argument("--tangent-gain", type=float, default=1.0)
    p.add_argument("--safety-gain", type=float, default=1.5, help="k_s (1/s)")
    p.add_argument("--a-max", type=float, default=3.0, help="deceleration the brake distance is sized for (m/s^2)")
    p.add_argument("--margin", type=float, default=None, help="s_margin of l_H (m); default 1 tangent, 3 brake")
    p.add_argument("--brake-speed", type=float, default=None,
                   help="v_eb (m/s); default 2 tangent (the authors' code), 0.3 brake")
    p.add_argument("--timeout", type=float, default=90.0, help="seconds of flight towards the goal at most")
    p.add_argument("--min-distance", type=float, default=1.0, help="pass if never closer than this to a wire (m)")
    p.add_argument("--rate", type=float, default=20.0, help="control rate (Hz)")
    p.add_argument("--keep", type=float, default=0.3,
                   help="seconds of clouds merged (the radars run at 10 Hz each, not in step)")
    p.add_argument("--max-lag", type=float, default=0.3,
                   help="hold while the newest cloud is older, by more than this (s), than the clouds of the first "
                        "second (simulation time against PX4's clock): the radar processing has fallen behind; 0: off")
    p.add_argument("--link", default="udpin:0.0.0.0:14540")
    p.add_argument("--no-land", action="store_true")
    p.add_argument("--demo", action="store_true",
                   help="for watching in RViz: keep flying (and hovering where the brake holds) until --timeout, "
                        "then stay in the air")
    args = p.parse_args()
    others = subprocess.run(["pgrep", "-f", "avoid_node.py"], capture_output=True, text=True).stdout.split()

    if any(int(pid) != os.getpid() for pid in others):
        # Two of these on one simulation both fly the vehicle (both bind the MAVLink port): run #901 was one
        # flight with two pilots, each logging its own result.
        sys.exit(f"another avoid_node.py is running (PID {', '.join(o for o in others if int(o) != os.getpid())}): "
                 f"stop it first")

    if args.demo:
        args.no_land = True

    # Tangent mode: r_a longer than the brake cone at cruise (5 m/s: l_H 5.2 m), so the tangent turns
    # the vehicle before the brake has to.
    tangent = args.mode == "tangent"
    args.avoid_radius = args.avoid_radius or (8.0 if tangent else 6.0)
    args.margin = args.margin if args.margin is not None else (1.0 if tangent else 3.0)
    args.brake_speed = args.brake_speed if args.brake_speed is not None else (2.0 if tangent else 0.3)
    rule = tangent_avoid if tangent else avoid
    # The authors' perception-horizon limit (offboard_control.cpp): no faster than the brake cone can be
    # filled by what the radars see, l_H = v^2 / 2 a_max + margin <= --detect-range.
    top_speed = math.sqrt(2.0 * args.a_max * max(args.detect_range - args.margin, 0.0)) if args.detect_range else math.inf
    mission = [np.array([float(c) for c in w.split(",")]) for w in args.waypoints] if args.waypoints else [np.array(args.goal)]

    import rclpy
    from geometry_msgs.msg import PointStamped
    from sensor_msgs.msg import PointCloud2

    sdf = os.path.join(ROOT, "worlds", args.world + ".sdf")
    wires = load_wires(sdf)
    obstacles = load_obstacles(sdf)
    params = Params(args)

    rclpy.init()
    node = rclpy.create_node("avoid_node")
    cloud = {"t": None, "points": np.empty((0, 3)), "n": 0, "received": 0}
    recent = []         # (time, points) of the last clouds: a radar frame that comes late or not at all
                        # must not empty the brake cone. In world axes, so older points are still in place.

    def on_cloud(msg):
        n = len(msg.data) // 24
        data = np.frombuffer(msg.data, dtype="<f4").reshape(n, 6)
        now = time.monotonic()
        cloud["received"] += 1
        recent.append((now, data[:, :3].astype(float)))

        while recent and now - recent[0][0] > args.keep:
            recent.pop(0)

        cloud["points"], cloud["t"] = np.vstack([pts for _, pts in recent]), now
        cloud["n"] = len(cloud["points"])

    node.create_subscription(PointCloud2, "/radar/points_map", on_cloud, 10)

    def on_seen_from(msg):
        cloud["seen_at"] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # When (simulation time) the radars saw the newest cloud (radar_array), against PX4's clock: in lockstep
    # PX4's time since boot is the simulation's time. Clouds arriving on time can still be old: a host busy
    # elsewhere left the radar processing seconds behind (runs #921-926) and the vehicle flew into wires not
    # yet seen. Not the vehicle's position against the cloud's: PX4's is an estimate, a metre off the true one
    # the radars use under the simulated GPS noise.
    node.create_subscription(PointStamped, "/radar/pose", on_seen_from, 10)
    view = LogicView(node, params)

    mav = mavutil.mavlink_connection(args.link)

    if mav.wait_heartbeat(timeout=20) is None:
        sys.exit("no MAVLink heartbeat: is the simulation running?")

    mav.mav.command_long_send(mav.target_system, mav.target_component,      # LOCAL_POSITION_NED at 50 Hz
                              mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 32, 20000, 0, 0, 0, 0, 0)

    state = {"pos": None, "vel": np.zeros(3)}

    def send(v_enu):
        # Velocity only, yaw held: ignore position, acceleration, yaw and yaw rate.
        mav.mav.set_position_target_local_ned_send(
            0, mav.target_system, mav.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527,
            0, 0, 0, float(v_enu[1]), float(v_enu[0]), float(-v_enu[2]), 0, 0, 0, 0, 0)

    def poll():
        # Every cloud waiting, not one per control step: six radars send ~60 a second.
        for _ in range(100):
            before = cloud["received"]
            rclpy.spin_once(node, timeout_sec=0.0)

            if cloud["received"] == before:
                break

        while True:
            m = mav.recv_match(type="LOCAL_POSITION_NED", blocking=False)

            if m is None:
                return

            state["pos"] = np.array([m.y, m.x, -m.z])
            state["time"] = m.time_boot_ms * 1e-3
            state["vel"] = np.array([m.vy, m.vx, -m.vz])

    period = 1.0 / args.rate

    def run_for(seconds, v=np.zeros(3)):
        end = time.time() + seconds

        while time.time() < end:
            poll()
            send(v)
            time.sleep(period)

    print("taking off")
    deadline = time.time() + 120

    while True:
        out = px4("commander", "takeoff")

        if out.returncode == 0 and "denied" not in (out.stdout + out.stderr).lower():
            break

        if time.time() > deadline:
            sys.exit(f"takeoff refused: {out.stdout.strip()} {out.stderr.strip()}")

        run_for(3)

    while state["pos"] is None or state["pos"][2] < 2.0:
        run_for(0.5)

        if time.time() > deadline:
            sys.exit("the vehicle did not climb")

    run_for(4)

    # Setpoints must already be flowing when PX4 is asked for Offboard.
    out = px4("commander", "mode", "offboard")
    run_for(1.5)
    nav = [l.split()[-1] for l in px4("listener", "vehicle_status", "-n", "1").stdout.splitlines()
           if l.strip().startswith("nav_state:")]
    print(f"commander mode offboard: {out.stdout.strip() or out.stderr.strip()}; nav_state {nav[0] if nav else '?'}")

    if not nav or nav[0] != "14":
        sys.exit("not in Offboard mode (nav_state 14): see the line above")

    leg = 0
    sent = np.zeros(3)
    lagging, lag_time, lags = False, 0.0, []
    goal = mission[0]
    rows, braking, climbed = [], False, False
    closest, closest_name = math.inf, None
    t0 = time.time()
    rest_since = None
    reached = False
    print(f"mode {args.mode}: flying {len(mission)} waypoint(s) {[tuple(w) for w in mission]}, "
          f"{len(wires)} wires, {len(obstacles)} obstacles known to the checker"
          + (f"; cruise capped at {top_speed:.1f} m/s (--detect-range {args.detect_range:g})" if top_speed < args.speed else ""))

    while time.time() - t0 < args.timeout:
        poll()
        pos, vel = state["pos"], state["vel"]

        if cloud["t"] is None or time.monotonic() - cloud["t"] > 1.0:
            send(np.zeros(3))               # no radar: stand still rather than fly blind
            time.sleep(period)
            continue

        lag = state["time"] - cloud["seen_at"] if "seen_at" in cloud and "time" in state else np.nan
        lags.append(lag)
        known = [l for l in lags if np.isfinite(l)]
        # The radars' own delay (scan period, processing, transport; 0.16-0.30 s on a free host) as the first
        # second shows it; what grows beyond it is the processing falling behind.
        baseline = min(float(np.median(known[:20])), BASELINE_MAX) if len(known) >= 20 else np.nan

        if args.max_lag > 0.0 and lag > baseline + args.max_lag:
            # The cloud is of a moment long gone: hold until the radars catch up.
            if not lagging:
                print(f"radar cloud {lag:.2f} s old ({baseline:.2f} s normal): holding", flush=True)

            lagging = True
            lag_time += period
            send(np.zeros(3))
            sent = np.zeros(3)
            time.sleep(period)
            continue

        lagging = False

        if not climbed and pos[2] >= goal[2] - 0.5:
            climbed = True

        # Climb first, straight up; then at the goal.
        target = goal if climbed else np.array([pos[0], pos[1], goal[2]])
        to_goal = target - pos
        d_goal = np.linalg.norm(to_goal)
        cruise = min(args.speed, top_speed) if climbed else args.climb_speed
        v_pilot = to_goal / max(d_goal, 1e-6) * min(cruise, 0.8 * d_goal)

        points = cloud["points"] - pos
        params.state["horizon"] = d_goal if climbed else np.inf
        debug = {}
        v_cmd, braking = rule(v_pilot, vel, points, params, braking, debug)

        overridden = braking or np.any(debug.get("push", np.zeros(3)))

        if args.cmd_accel and not overridden:
            # Speed up gradually, as the authors' code smooths its output: a step to cruise speed tilts the
            # vehicle 35-40 deg nose down, and a wire ahead at its height leaves the fields of view (run #122).
            # Only the speed: a turn away from an obstacle goes out at once (ramping it too let the vehicle
            # carry on into wire_a, run #442), as do the brake and the safety push.
            limit = np.linalg.norm(sent) + args.cmd_accel * period
            norm = np.linalg.norm(v_cmd)

            if norm > limit:
                v_cmd = v_cmd * limit / norm

        sent = vel.copy() if overridden else v_cmd       # after the brake, ramp on from the actual velocity
        send(v_cmd)

        if len(rows) % 2 == 0:              # 10 Hz is enough to watch
            view.publish(pos, vel, v_pilot, v_cmd, braking, points, debug)

        d_wire, name = distance_to_segments(pos, wires)
        d_obs, obs_name = min(((float(o.surface_distance(pos)[0]), o.name) for o in obstacles), default=(math.inf, None))
        d_true = min(d_wire, d_obs)

        if d_true < closest:
            closest, closest_name = d_true, name if d_wire <= d_obs else obs_name

        d_cloud = float(np.linalg.norm(points, axis=1).min()) if len(points) else math.inf
        shift = debug.get("shift", np.zeros(3))
        rows.append((time.time() - t0, *pos, *vel, *v_cmd, int(braking), cloud["n"], d_cloud, d_wire, d_obs,
                     *shift, params.state.pop("gap", np.nan), lag))

        speed = np.linalg.norm(vel)

        if climbed and d_goal < (args.accept if leg < len(mission) - 1 else 0.5):
            print(f"waypoint {leg + 1}/{len(mission)} reached at ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})", flush=True)
            leg += 1

            if leg == len(mission):
                print("goal reached")
                reached = True
                break

            goal = mission[leg]

        if climbed and speed < 0.2 and braking and not args.demo:
            rest_since = rest_since or time.time()

            if time.time() - rest_since > 5.0:
                print("held by the emergency brake")
                break
        else:
            rest_since = None

        if d_true < 0.3:
            print("collision")
            break

        time.sleep(period)

    run_for(1)

    if not args.no_land:
        px4("commander", "land")

    os.makedirs(os.path.join(ROOT, "out"), exist_ok=True)

    with open(os.path.join(ROOT, "out", "avoid.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "cmd_x", "cmd_y", "cmd_z", "brake", "points",
                    "d_cloud", "d_wire", "d_obstacle", "shift_x", "shift_y", "shift_z", "line_gap", "radar_lag"])
        w.writerows(rows)

    if not rows:
        known = [l for l in lags if np.isfinite(l)]

        if lag_time > 0.0 and known:
            # Held the whole time (run #972: clouds 18 s old under a loaded host): no test of the avoidance.
            sys.exit(f"no flight: held {lag_time:.0f} s, radar clouds up to {max(known):.1f} s old  "
                     f"NOT VALID: the radar processing fell behind")

        sys.exit("no flight data (no /radar/points?)")

    # Tangent mode is meant to get through: the mission must be finished too.
    passed = closest >= args.min_distance and (reached or not tangent or args.proximity_test)
    known = np.array([l for l in lags if np.isfinite(l)])
    lag_line = "radar lag unknown (no /radar/pose)"

    if len(known):
        normal = float(np.median(known[:20]))
        late = np.percentile(known, 95)
        # A flight on clouds this old was not a test of the avoidance (runs #921-926): say so.
        lag_line = (f"radar lag {np.median(known):.2f} s median, {late:.2f} s 95 %, {known.max():.2f} s max "
                    f"({normal:.2f} s in the first second)"
                    + ("  NOT VALID: the radar processing fell behind" if late > normal + 0.2 or normal > BASELINE_MAX
                       else ""))
    final = np.array(rows[-1][1:4])
    peak = max(np.linalg.norm(r[4:7]) for r in rows)
    print(f"\nfinal    ({final[0]:.1f}, {final[1]:.1f}, {final[2]:.1f}), {np.linalg.norm(rows[-1][4:7]):.2f} m/s\n"
          f"speed    {peak:.1f} m/s at most\nclosest  {closest:.2f} m to {closest_name} "
          f"(wire radius subtracted)\nmission  {'finished' if reached else f'{leg}/{len(mission)} waypoints'}\n"
          f"held     {lag_time:.1f} s for radar lag\n{lag_line}\nresult:  {'PASS' if passed else 'FAIL'}")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
