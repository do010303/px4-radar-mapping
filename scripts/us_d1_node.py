#!/usr/bin/env python3
"""Ainstein US-D1 on a serial line: the emulated radar as the packets the real one sends.

Turns every Gazebo scan of the radar (profiles/us_d1.yaml, output: altimeter) into
the US-D1 UART packet of its manual (D00.02.05, 5.1) and writes it to a
pseudo-terminal, at the sensor's rate. PX4's own ulanding_radar driver reads it
from the symlink --tty, so the detection path through PX4 is the real one:

    gz scan -> radar model -> 6-byte packet -> pty -> ulanding_radar -> distance_sensor

Packet: 0xFE, 0x02 (version), altitude LSB, altitude MSB (cm), SNR (dB),
checksum = (version + altitude + SNR) & 0xFF.

With nothing in the beam the real sensor reports about 0.51 m, the edge of its blind
zone (manual 11, issue 1); its SNR then is not stated, so a low one is assumed.

    scripts/us_d1_node.py --world radar_hover --model x500_radar_0
"""

import argparse
import errno
import os
import pty
import sys
import threading
import time
import tty

import numpy as np

from radar_model import add_model_args, detect, scan_to_grid    # first: sets up the gz bindings
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.transport13 import Node

HEADER, VERSION = 0xFE, 0x02
# No return at all: the blind-zone edge and an SNR well below the 13 dB limit (estimate).
NO_RETURN_CM, NO_RETURN_SNR = 51, 3


def packet(det):
    """The 6 bytes for one frame: det is the (N, 5) detections of the radar model, N <= 1."""
    if len(det):
        cm = int(np.clip(round(np.linalg.norm(det[0, :3]) * 100.0), 0, 0xFFFF))
        snr = int(np.clip(round(det[0, 4]), 0, 0xFF))
    else:
        cm, snr = NO_RETURN_CM, NO_RETURN_SNR

    lo, hi = cm & 0xFF, cm >> 8
    return bytes((HEADER, VERSION, lo, hi, snr, (VERSION + lo + hi + snr) & 0xFF))


def open_tty(link):
    """A raw pty; link becomes a symlink to its slave side, for the PX4 driver to open."""
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    tmp = link + ".tmp"
    os.makedirs(os.path.dirname(os.path.abspath(link)), exist_ok=True)

    if os.path.lexists(tmp):
        os.remove(tmp)

    os.symlink(os.ttyname(slave), tmp)
    os.replace(tmp, link)
    return master, slave


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--world", default="radar_hover")
    p.add_argument("--model", default="x500_radar_0", help="Gazebo model carrying the radar")
    p.add_argument("--link", default="radar_link", help="radar link inside that model")
    p.add_argument("--tty", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "run", "us_d1_0"),
                   help="symlink to the pseudo-terminal the packets are written to")
    p.add_argument("--sensors", help="comma-separated sides of models/x500_us_d1 (left,center,right): one radar and one "
                                     "pseudo-terminal each, <dir of --tty>/us_d1_<n> in this order; replaces --link")
    p.add_argument("--max-lag", type=float, default=0.1,
                   help="drop scans this much (s, simulation time) older than the newest one of any radar")
    p.add_argument("--seed", type=int, default=1)
    add_model_args(p)
    args = p.parse_args()

    if args.output != "altimeter":
        sys.exit(f"us_d1_node: the active radar profile has output '{args.output}', not 'altimeter'; "
                 "build the model with scripts/make_radar_model.py profiles/us_d1.yaml")

    if args.sensors:
        tty_dir = os.path.dirname(os.path.abspath(args.tty))
        sensors = [(f"radar_link_us_d1_{side}", f"/radar/scan_us_d1_{side}", os.path.join(tty_dir, f"us_d1_{i}"))
                   for i, side in enumerate(args.sensors.split(","))]
    else:
        sensors = [(args.link, "/radar/scan", args.tty)]

    threshold = args.snr_threshold
    counts, fds = [], []
    node = Node()
    lock = threading.Lock()
    newest = {"stamp": 0.0}

    for i, (link, topic, tty_link) in enumerate(sensors):
        master, slave = open_tty(tty_link)
        fds += [master, slave]
        count = {"frames": 0, "valid": 0, "dropped": 0, "stale": 0}
        counts.append(count)

        def on_scan(msg, master=master, count=count, rng=np.random.default_rng(args.seed + i)):
            # Only the ray grid: the packet carries no Doppler, so the radar's pose is not needed.
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

            with lock:
                newest["stamp"] = max(newest["stamp"], stamp)
                lag = newest["stamp"] - stamp

            # A scan older than another radar's newest one means this process is behind: drop it
            # rather than send PX4 an old reading (it once sent ground returns from before take-off).
            if lag > args.max_lag:
                count["stale"] += 1
                return

            ranges, az, el = scan_to_grid(msg)
            det = detect(ranges, az, el, args, rng)
            count["frames"] += 1
            count["valid"] += int(len(det) > 0 and det[0, 4] > threshold)

            try:
                os.write(master, packet(det))
            except OSError as e:
                # Nobody has opened the other end yet, or it does not keep up: a real UART drops bytes too.
                if e.errno not in (errno.EAGAIN, errno.EIO):
                    raise

                count["dropped"] += 1

        if not node.subscribe(LaserScan, topic, on_scan):
            sys.exit(f"us_d1_node: failed to subscribe to {topic}")

        print(f"us_d1_node: {link} on {topic}: packets on {os.path.realpath(tty_link)} (link {os.path.abspath(tty_link)})",
              flush=True)

    def status():
        last = time.monotonic()

        while True:
            time.sleep(1.0)

            if time.monotonic() - last >= 10.0:
                last = time.monotonic()

                for (link, _, _), count in zip(sensors, counts):
                    n = max(count["frames"], 1)
                    print(f"us_d1_node: {link}: {count['frames']} packets, {100 * count['valid'] / n:.0f}% above "
                          f"{threshold:g} dB, {count['dropped']} dropped, {count['stale']} stale scans skipped", flush=True)

    threading.Thread(target=status, daemon=True).start()

    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        for fd in fds:
            os.close(fd)

        for _, _, tty_link in sensors:
            if os.path.islink(tty_link):
                os.remove(tty_link)


if __name__ == "__main__":
    main()
    sys.stdout.flush()
    os._exit(0)
