#!/usr/bin/env python3
"""Collision prevention test: fly at a wall in Position mode and see where PX4 stops.

Against a running  CP_DIST=5 US_D1=3 PX4_GZ_WORLD=us_d1_wall ./run_sitl.sh  (HEADLESS=1 PX4_DAEMON=1
for no windows): takes off, switches to Position mode, then holds the pitch stick forward over MAVLink
(MANUAL_CONTROL) so the vehicle flies at worlds/us_d1_wall.sdf's wall. PX4's collision prevention
(src/lib/collision_prevention) works from the distance_sensor topics of the us_d1_radar driver; it
slows and stops the vehicle CP_DIST from the obstacle. Writes out/cp_test.csv and reports:

  stopped at   how far from the wall the vehicle came to rest (m, from Gazebo's frame: the wall is at x = 40)
  closest      the smallest distance to the wall during the run
  speed        the forward speed before the slowdown

Exit codes: 0 the vehicle stopped at least --min-distance from the wall, 1 it came closer or did not stop,
2 setup problem.
"""

import argparse
import csv
import os
import subprocess
import sys
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PX4_DIR = os.path.normpath(os.path.join(ROOT, "..", ".."))
_OWN = os.path.join(PX4_DIR, "build", "px4_sitl_mapping")
PX4_BIN = os.path.join(os.environ.get("PX4_BUILD") or (_OWN if os.path.exists(os.path.join(_OWN, "bin", "px4"))
                                                       else os.path.join(PX4_DIR, "build", "px4_sitl_default")), "bin")

WALL_X = 40.0 - 0.2          # the near face of worlds/us_d1_wall.sdf's wall, Gazebo x (--wall-x)
SIDES = {7: "left", 0: "ahead", 1: "right"}     # PX4 sensor rotation of the three radars
NOSE = 0.15                  # the sensors sit about this far ahead of the vehicle origin


def px4(*cmd):
    return subprocess.run([os.path.join(PX4_BIN, f"px4-{cmd[0]}"), *cmd[1:]], capture_output=True, text=True, timeout=20)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--stick", type=int, default=300, help="forward stick, 0..1000 (1000 = full)")
    p.add_argument("--timeout", type=float, default=60.0, help="seconds of forward stick at most")
    p.add_argument("--min-distance", type=float, default=3.0, help="pass if the vehicle stops at least this far (m)")
    p.add_argument("--wall-x", type=float, default=WALL_X,
                   help="Gazebo x of the face the vehicle flies at: 39.8 the wall, 27.95 the pole of us_d1_sides")
    p.add_argument("--link", default="udpin:0.0.0.0:14550")
    args = p.parse_args()

    mav = mavutil.mavlink_connection(args.link)

    if mav.wait_heartbeat(timeout=20) is None:
        sys.exit("no MAVLink heartbeat: is the simulation running?")

    stick = {"x": 0}
    last = {"pos": None}
    readings = {}               # sensor rotation -> distances (m) seen while the stick is forward
    newest = {}                 # sensor rotation -> its latest reading (m, None when invalid)
    log = {"on": False}

    def pump(seconds):
        """Send the sticks at 20 Hz for this long, keeping the newest LOCAL_POSITION_NED.

        One thread does all the MAVLink I/O: PX4 only keeps Position mode while the stick stream is
        steady, and a separate sender thread stalled behind the blocking receive.
        """
        end = time.time() + seconds

        while time.time() < end:
            mav.mav.manual_control_send(mav.target_system, stick["x"], 0, 500, 0, 0)
            m = mav.recv_match(type=["LOCAL_POSITION_NED", "DISTANCE_SENSOR"], blocking=True, timeout=0.05)

            if m is None:
                continue

            if m.get_type() == "LOCAL_POSITION_NED":
                last["pos"] = m
            else:
                valid = m.current_distance > 0 and getattr(m, "signal_quality", 100) > 0
                newest[m.orientation] = m.current_distance / 100.0 if valid else None

                if log["on"]:
                    readings.setdefault(m.orientation, []).append(newest[m.orientation])

    cp = px4("param", "show", "CP_DIST").stdout
    print("CP_DIST:", cp.strip().splitlines()[-1] if cp.strip() else "?")

    # LOCAL_POSITION_NED at 20 Hz, whatever the link's default stream.
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 32, 50000, 0, 0, 0, 0, 0)
    mav.mav.command_long_send(mav.target_system, mav.target_component,        # DISTANCE_SENSOR at 10 Hz
                              mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 132, 100000, 0, 0, 0, 0, 0)

    # One takeoff command: each further one climbs another takeoff altitude.
    print("taking off")
    deadline = time.time() + 120

    while True:
        out = px4("commander", "takeoff")

        if out.returncode == 0 and "denied" not in (out.stdout + out.stderr).lower():
            break

        if time.time() > deadline:
            sys.exit(f"takeoff refused: {out.stdout.strip()} {out.stderr.strip()}")

        pump(3)

    while last["pos"] is None or -last["pos"].z < 2.0:
        pump(0.5)

        if time.time() > deadline:
            sys.exit("the vehicle did not climb")

    pump(6)
    print(f"hovering at {-last['pos'].z:.1f} m")

    # Position mode, and make sure it took: in Takeoff or Hold the sticks do nothing and
    # collision prevention does not run.
    out = px4("commander", "mode", "posctl")
    pump(2)
    status = px4("listener", "vehicle_status", "-n", "1").stdout
    nav = [l.split()[-1] for l in status.splitlines() if l.strip().startswith("nav_state:")]
    print(f"commander mode posctl: {out.stdout.strip() or out.stderr.strip()}; nav_state {nav[0] if nav else '?'}")

    if not nav or nav[0] != "2":
        sys.exit("not in Position mode (nav_state 2): see the line above")

    rows = []
    t0 = time.time()
    stick["x"] = args.stick
    log["on"] = True
    print(f"stick forward {args.stick}/1000")

    while time.time() - t0 < args.timeout:
        pump(0.05)
        m = last["pos"]

        if m is None:
            continue

        gz_x = m.y                  # NED east is Gazebo +X: the vehicle faces it at take-off
        dist = args.wall_x - (gz_x + NOSE)
        rows.append((time.time() - t0, gz_x, m.vy, m.z * -1, dist, *(newest.get(r) for r in SIDES)))

        if dist < 1.0:
            print("too close to the wall: sticks neutral")
            break

        if len(rows) > 100 and abs(rows[-1][2]) < 0.1 and time.time() - t0 > 8 and rows[-1][1] > 5:
            break               # at rest after having flown

    stick["x"] = 0
    pump(1)
    px4("commander", "land")

    os.makedirs(os.path.join(ROOT, "out"), exist_ok=True)

    with open(os.path.join(ROOT, "out", "cp_test.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "gz_x", "v_forward", "height", "distance_to_wall", *(f"d_{n}" for n in SIDES.values())])
        w.writerows(rows)

    if not rows:
        sys.exit("no position data")

    for rotation, name in SIDES.items():
        seen = readings.get(rotation, [])
        valid = [d for d in seen if d is not None]
        print(f"radar {name:<5} {len(seen):4d} readings, {100 * len(valid) / max(len(seen), 1):3.0f}% valid, "
              f"nearest {min(valid):.1f} m" if valid else f"radar {name:<5} {len(seen):4d} readings, none valid")

    cruise = max(abs(r[2]) for r in rows)
    closest = min(r[4] for r in rows)
    final = rows[-1][4]
    print(f"\nspeed    {cruise:.1f} m/s at most\nclosest  {closest:.1f} m to the wall\nstopped at {final:.1f} m "
          f"(rest: {abs(rows[-1][2]) < 0.2})")
    ok = closest >= args.min_distance and abs(rows[-1][2]) < 0.3
    print("result:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
