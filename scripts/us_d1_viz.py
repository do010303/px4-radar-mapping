#!/usr/bin/env python3
"""RViz view of the three US-D1 radars as PX4 sees them: reads PX4's own MAVLink stream.

Publishes, from PX4's onboard MAVLink port (udp 14540), so it shows what collision prevention works
from and costs the simulation nothing:

  TF map -> base_link          from LOCAL_POSITION_NED and ATTITUDE
  /us_d1/markers (MarkerArray) per radar: its -3 dB field of view (as mounted, with the tilt) and
                               its reading (DISTANCE_SENSOR): a line and a sphere at the measured
                               distance, green when valid, red when below the quality limit;
                               and PX4's 72-bin obstacle map (OBSTACLE_DISTANCE, 5 deg bins) as points.

    source env.sh && ./scripts/us_d1_viz.py            # PX4 SITL from run_sitl.sh, US_D1=3
    rviz2 -d rviz/us_d1.rviz
"""

import argparse
import os
import sys
import time

import numpy as np
from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from radar_model import fov_segments

# Mounts of models/x500_us_d1*: PX4 rotation -> (position in base_link FLU, yaw deg).
MOUNTS = {7: ((0.10, 0.05, -0.06), 45.0), 0: ((0.12, 0.0, -0.06), 0.0), 1: ((0.10, -0.05, -0.06), -45.0)}
NAMES = {7: "left", 0: "ahead", 1: "right"}
QUALITY_MIN = 1

R_ENU_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
R_FRD_FLU = np.diag([1.0, -1.0, -1.0])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def to_quaternion(r):
    """(x, y, z, w) of a rotation matrix."""
    w = np.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = np.copysign(np.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    return float(x), float(y), float(z), float(w)


def sensor_rotation(yaw_deg, tilt_deg):
    """Sensor-to-body rotation in FLU: yawed, then tilted up (a positive Y rotation points the nose down)."""
    return rot_z(np.radians(yaw_deg)) @ rot_y(-np.radians(tilt_deg))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--link", default="udpin:0.0.0.0:14540", help="PX4's onboard MAVLink port")
    p.add_argument("--tilt-deg", type=float, default=15.0, help="upward tilt of the radars (x500_us_d1_up15: 15, x500_us_d1: 0)")
    p.add_argument("--fov-range", type=float, default=50.0, help="range at which the field of view is drawn (m)")
    p.add_argument("--beam-az", type=float, default=43.0)
    p.add_argument("--beam-el", type=float, default=30.0)
    args = p.parse_args()

    import rclpy
    from geometry_msgs.msg import Point, TransformStamped
    from std_msgs.msg import ColorRGBA
    from tf2_ros import TransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init()
    node = rclpy.create_node("us_d1_viz")
    pub = node.create_publisher(MarkerArray, "/us_d1/markers", 10)
    tf = TransformBroadcaster(node)

    mav = mavutil.mavlink_connection(args.link)
    print(f"us_d1_viz: waiting for PX4 on {args.link}", flush=True)
    mav.wait_heartbeat()

    for msg_id, hz in ((32, 20), (30, 20), (132, 10), (330, 10)):     # LOCAL_POSITION_NED, ATTITUDE, DISTANCE_SENSOR, OBSTACLE_DISTANCE
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    state = {"pos": np.zeros(3), "att": (0.0, 0.0, 0.0), "dist": {}, "map": None}
    az, el = np.radians(args.beam_az / 2), np.radians(args.beam_el / 2)
    fov = {}

    for rotation, (pos, yaw) in MOUNTS.items():
        r = sensor_rotation(yaw, args.tilt_deg)
        segs = fov_segments(args.fov_range, -az, az, -el, el)       # (M, 2, 3) in the sensor frame
        fov[rotation] = (np.asarray(pos) + segs @ r.T)

    def marker(mid, ns, kind, scale, color):
        m = Marker()
        m.header.frame_id = "base_link"
        m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
        m.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(color[3]))
        return m

    def publish():
        stamp = node.get_clock().now().to_msg()
        roll, pitch, yaw = state["att"]
        r_enu_flu = R_ENU_NED @ (rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)) @ R_FRD_FLU
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "map", "base_link"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, state["pos"])
        (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
         t.transform.rotation.w) = to_quaternion(r_enu_flu)
        tf.sendTransform(t)

        out = MarkerArray()

        for k, (rotation, segs) in enumerate(fov.items()):
            m = marker(k, "us_d1_fov", Marker.LINE_LIST, (0.03, 0, 0), (0.95, 0.85, 0.25, 0.8))
            m.points = [Point(x=float(q[0]), y=float(q[1]), z=float(q[2])) for q in segs.reshape(-1, 3)]
            out.markers.append(m)

            pos, yaw = MOUNTS[rotation]
            reading = state["dist"].get(rotation)
            line = marker(10 + k, "us_d1_reading", Marker.LINE_LIST, (0.04, 0, 0), (0.2, 0.9, 0.3, 1.0))
            ball = marker(20 + k, "us_d1_reading", Marker.SPHERE, (0.8, 0.8, 0.8), (0.2, 0.9, 0.3, 1.0))
            text = marker(30 + k, "us_d1_label", Marker.TEXT_VIEW_FACING, (0, 0, 0.6), (1, 1, 1, 1))

            if reading is not None:
                d, quality = reading
                valid = d >= 0 and quality >= QUALITY_MIN
                color = (0.2, 0.9, 0.3, 1.0) if valid else (0.95, 0.25, 0.2, 1.0)
                line.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
                ball.color = line.color
                shown = d if valid else args.fov_range
                end = np.asarray(pos) + sensor_rotation(yaw, args.tilt_deg) @ np.array([shown, 0, 0])
                line.points = [Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
                               Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))]
                ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = map(float, end)
                text.pose.position.x, text.pose.position.y, text.pose.position.z = float(end[0]), float(end[1]), float(end[2]) + 0.8
                text.text = f"{NAMES[rotation]}: {d:.1f} m" if valid else f"{NAMES[rotation]}: no obstacle"

            out.markers += [line, ball, text]

        cloud = marker(40, "us_d1_map", Marker.POINTS, (0.35, 0.35, 0), (1.0, 0.55, 0.1, 1.0))

        if state["map"] is not None:
            distances, increment, offset = state["map"]

            for i, d in enumerate(distances):
                if d < 65535:
                    a = np.radians(offset + (i + 0.5) * increment)       # clockwise from ahead (FRD)
                    cloud.points.append(Point(x=float(d / 100 * np.cos(a)), y=float(-d / 100 * np.sin(a)), z=0.0))

        out.markers.append(cloud)
        pub.publish(out)

    print("us_d1_viz: publishing /us_d1/markers and TF map -> base_link", flush=True)
    last = 0.0

    while True:
        m = mav.recv_match(blocking=True, timeout=0.1)

        if m is not None:
            kind = m.get_type()

            if kind == "LOCAL_POSITION_NED":
                state["pos"] = np.array([m.y, m.x, -m.z])          # NED -> ENU
            elif kind == "ATTITUDE":
                state["att"] = (m.roll, m.pitch, m.yaw)
            elif kind == "DISTANCE_SENSOR":
                state["dist"][m.orientation] = (m.current_distance / 100.0, getattr(m, "signal_quality", 100))
            elif kind == "OBSTACLE_DISTANCE":
                state["map"] = (list(m.distances), m.increment or m.increment_f, m.angle_offset)

        if time.time() - last > 0.1:
            last = time.time()
            publish()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
