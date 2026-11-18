#!/usr/bin/env python3
"""Show the emulated radar live, in the Gazebo GUI and in ROS 2 / RViz.

Gazebo GUI: the field of view is a wireframe visual of the radar model itself
(models/radar_mmwave), so it needs nothing from here. With --gz-points, every
detection is also drawn as a small sphere (markers, world frame), orange when
it lies on the ground (a plane fitted to the detections below the radar,
scene.fit_ground), blue otherwise. Off by default: a few hundred markers
redrawn several times a second slow the GUI down, and RViz shows the same.

ROS 2 (for RViz, see rviz/radar.rviz):
  * /radar/points  sensor_msgs/PointCloud2 in frame radar_link, with fields
                   x, y, z, velocity (radial, m/s) and snr (dB);
  * /radar/fov     visualization_msgs/Marker, the field-of-view frustum;
  * TF map -> radar_link from the Gazebo ground truth.

    ./radar_viz.py --world radar_hover --model x500_radar_0     # vehicle in PX4 SITL
    ./radar_viz.py --world radar_test --model radar_stand       # static bench
    ./radar_viz.py ... --gz-points                              # detections in Gazebo too
    ./radar_viz.py ... --gz-points --no-ros                     # Gazebo markers only
"""

import argparse
import multiprocessing
import os
import queue
import sys
import threading
import time

import numpy as np

from radar_model import add_model_args, display_angles, fov_segments, load_profile    # first: sets up the gz bindings
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.transport13 import Node
from radar_stream import RadarStream
from scene import fit_ground

NS = "px4_mapping_radar"
GROUND_TOLERANCE = 0.3  # m from the fitted ground plane that still counts as ground
BLUE = (0.16, 0.47, 0.84)
ORANGE = (0.92, 0.41, 0.20)
FOV_COLOR = (0.95, 0.85, 0.25)


def send_markers(batches):
    """Sender process: passes marker batches on to the Gazebo GUI.

    A request from the Python binding holds the interpreter while it waits; a
    scan arriving in that moment blocks the transport's receive thread and the
    answer is never read. A process that subscribes to nothing avoids that.
    """
    node = Node()
    parent = os.getppid()

    while os.getppid() == parent:
        try:
            data = batches.get(timeout=1.0)
        except queue.Empty:
            continue

        if data is None:
            break

        batch = Marker_V()
        batch.ParseFromString(data)
        node.request("/marker_array", batch, Marker_V, Boolean, 1000)


def set_color(marker, rgb, alpha=1.0):
    # Emissive only: the colour then does not depend on the light direction.
    for c, gain in ((marker.material.ambient, 0.0), (marker.material.diffuse, 0.0),
                    (marker.material.emissive, 1.0)):
        c.r, c.g, c.b, c.a = gain * rgb[0], gain * rgb[1], gain * rgb[2], alpha


class GazeboDrawer:
    def __init__(self, args):
        self.args = args
        self.drawn = 0
        self.last = 0.0
        context = multiprocessing.get_context("spawn")
        self.batches = context.Queue(maxsize=4)
        self.sender = context.Process(target=send_markers, args=(self.batches,), daemon=True)
        self.sender.start()

    def draw(self, frame):
        now = time.monotonic()

        if now - self.last < 1.0 / self.args.gz_rate:
            return

        self.last = now
        batch = Marker_V()

        points = frame.world_points()
        plane = fit_ground(points, frame.position)
        ground = np.zeros(len(points), dtype=bool) if plane is None else \
            np.abs(points @ plane[0] + plane[1]) < GROUND_TOLERANCE

        for i, xyz in enumerate(points):
            m = batch.marker.add()
            m.ns, m.id, m.action, m.type = NS, i, Marker.ADD_MODIFY, Marker.SPHERE
            m.visibility = Marker.GUI
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in xyz)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.args.point_size
            set_color(m, ORANGE if ground[i] else BLUE)

        for i in range(len(points), self.drawn):
            m = batch.marker.add()
            m.ns, m.id, m.action = NS, i, Marker.DELETE_MARKER

        self.drawn = len(points)

        try:
            self.batches.put_nowait(batch.SerializeToString())
        except queue.Full:
            pass        # the GUI is behind; skip this frame rather than queue up lag

    def close(self):
        batch = Marker_V()
        m = batch.marker.add()
        m.ns, m.action = NS, Marker.DELETE_ALL

        try:
            self.batches.put(batch.SerializeToString(), timeout=1.0)
            self.batches.put(None, timeout=1.0)
        except queue.Full:
            pass

        self.sender.join(timeout=3.0)


class RosPublisher:
    def __init__(self, args):
        import rclpy
        from geometry_msgs.msg import Point, TransformStamped
        from sensor_msgs.msg import PointCloud2, PointField
        from tf2_ros import TransformBroadcaster
        from visualization_msgs.msg import Marker as RosMarker

        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("radar_viz")
        self.cloud_pub = self.node.create_publisher(PointCloud2, args.cloud_topic, 10)
        self.fov_pub = self.node.create_publisher(RosMarker, "/radar/fov", 10)
        self.tf = TransformBroadcaster(self.node)
        self.args = args
        self.PointCloud2, self.TransformStamped = PointCloud2, TransformStamped
        self.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                       for i, n in enumerate(("x", "y", "z", "velocity", "snr"))]

        fov = RosMarker()
        fov.header.frame_id = args.radar_frame
        fov.ns, fov.id, fov.type, fov.action = "radar_fov", 0, RosMarker.LINE_LIST, RosMarker.ADD
        fov.pose.orientation.w = 1.0
        fov.scale.x = 0.02
        fov.color.r, fov.color.g, fov.color.b, fov.color.a = (*FOV_COLOR, 0.8)
        self.Point = Point
        self.fov = fov

    def publish(self, frame, angles):
        if not self.fov.points:
            # The field of view comes from the scan, so it matches the sensor's SDF.
            self.fov.points = [self.Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                               for p in fov_segments(self.args.fov_range, *display_angles(load_profile(), angles)).reshape(-1, 3)]

        # Wall-clock stamps: RViz has no /clock here, and TF and cloud only need to agree.
        stamp = self.node.get_clock().now().to_msg()

        t = self.TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.args.map_frame
        t.child_frame_id = self.args.radar_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            (float(v) for v in frame.position)
        w, x, y, z = matrix_to_quat(frame.rotation)
        t.transform.rotation.w, t.transform.rotation.x = w, x
        t.transform.rotation.y, t.transform.rotation.z = y, z
        self.tf.sendTransform(t)

        cloud = self.PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.args.radar_frame
        cloud.height, cloud.width = 1, len(frame.det)
        cloud.fields = self.fields
        cloud.point_step, cloud.row_step = 20, 20 * len(frame.det)
        cloud.is_dense = True
        cloud.data = frame.det.astype("<f4").tobytes()
        self.cloud_pub.publish(cloud)

        self.fov.header.stamp = stamp
        self.fov_pub.publish(self.fov)

    def spin(self):
        self.rclpy.spin(self.node)


def matrix_to_quat(r):
    w = np.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = np.copysign(np.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    return float(w), float(x), float(y), float(z)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--world", default="radar_hover")
    p.add_argument("--model", default="x500_radar_0", help="Gazebo model carrying the radar")
    p.add_argument("--link", default="radar_link", help="radar link inside that model")
    add_model_args(p)
    p.add_argument("--fov-range", type=float,
                   default=float(load_profile().get("display", {}).get("fov_range", 4.0)),
                   help="range at which RViz draws the field of view (m), as the Gazebo wireframe")
    p.add_argument("--point-size", type=float, default=0.15, help="detection sphere diameter in Gazebo (m)")
    p.add_argument("--gz-rate", type=float, default=5.0, help="Gazebo marker updates per second")
    p.add_argument("--gz-points", action="store_true",
                   help="also draw the detections into the Gazebo GUI (slows it down; RViz shows them)")
    p.add_argument("--no-ros", action="store_true", help="do not publish to ROS 2")
    p.add_argument("--cloud-topic", default="/radar/points")
    p.add_argument("--map-frame", default="map")
    p.add_argument("--radar-frame", default="radar_link")
    p.add_argument("--stall-timeout", type=float, default=2.0,
                   help="seconds without a radar scan before warning that the radar stalled")
    p.add_argument("--seed", type=int, default=1)
    args = p.parse_args()

    gz = GazeboDrawer(args) if args.gz_points else None
    ros = None if args.no_ros else RosPublisher(args)
    count = {"frames": 0, "points": 0}

    def on_frame(frame):
        count["frames"] += 1
        count["points"] += len(frame.det)

        if gz:
            gz.draw(frame)

        if ros:
            ros.publish(frame, stream.angles)

    stream = RadarStream(args, args.world, args.model, on_frame, link=args.link, seed=args.seed)
    print(f"radar_viz: world {args.world}, model {args.model}"
          f"{', Gazebo markers' if args.gz_points else ''}{'' if args.no_ros else ', ROS 2 ' + args.cloud_topic}",
          flush=True)

    def status():
        """Status every 10 s, and a warning as soon as the radar stops sending scans."""
        stalled_since = None
        last_report = time.monotonic()

        while True:
            time.sleep(0.5)
            now = time.monotonic()
            last = stream.last_scan

            if last is not None and now - last > args.stall_timeout:
                if stalled_since is None:
                    stalled_since = last
                    print(f"radar_viz: WARNING radar stalled, no scan for {now - last:.1f} s", flush=True)
            elif stalled_since is not None:
                print(f"radar_viz: radar back after {now - stalled_since:.1f} s", flush=True)
                stalled_since = None

            if now - last_report >= 10.0:
                last_report = now
                n = max(count["frames"], 1)
                print(f"radar_viz: {count['frames']} frames, {count['points'] / n:.0f} detections/frame, "
                      f"{stream.scans_without_pose} scans without a pose", flush=True)

    threading.Thread(target=status, daemon=True).start()

    try:
        if ros:
            ros.spin()
        else:
            threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        if gz:
            gz.close()


if __name__ == "__main__":
    main()
    # Skip interpreter teardown: destroying the gz-transport node while its
    # threads still deliver messages aborts the process.
    sys.stdout.flush()
    os._exit(0)
