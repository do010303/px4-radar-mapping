#!/bin/bash
# The real-time radar map live (docs/survey_mapping_plan.md, phase 5): the survey flight over
# worlds/survey_site with the Gazebo GUI, the on-board mapper on the radar frames and PX4's MAVLink pose,
# the 300 kbit/s link to the ground map, shown in RViz (rviz/ground.rviz).
#
#   scripts/survey_live.sh [tag] [kbps]        default: live_1 300
#   HEADLESS=1 scripts/survey_live.sh          without the Gazebo GUI (RViz still opens)
#   HEADLESS=1 VIEW=0 scripts/survey_live.sh   nothing on screen: the headless check before a GUI run
#   GPS=std scripts/survey_live.sh std_5      with standard GPS instead of RTK (run_sitl.sh GPS=)
#
# Starts and stops only its own processes, by PID; run_sitl.sh waits for other simulations as always.
# Logs in run/{ground_map,onboard_live}.log; the flight in out/survey/<tag>.*.

here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$here"
. ./env.sh
tag=${1:-live_1} kbps=${2:-300}
# Surfaces as a surface larger than the radar cell returns (R^-2) and brighter by 8 dB: with the profiles'
# defaults (R^-4, set for the wires) a 6 x 4 m wall face-on vanished beyond 8 m for an AOP, before a 20 mm
# wire did (docs/survey_mapping_plan.md, phase 5); an estimate until real radar data. SURFACE= : the defaults.
SURFACE=${SURFACE---surface-law 20 --surface-db 8}
mine=()
sim=""          # this run's simulation PIDs: run/pids as it was once its SITL was ready

# One run at a time: they share run/ (pids, logs) and the UDP ports. A second one waits for the first to end
# (it used to start in the first's last seconds, and the first, stopping by run/pids, stopped the second's sim).
exec 9> run/survey_live.lock
if ! flock -n 9; then
	echo "another survey_live.sh is running: waiting for it to end"
	flock 9
fi

stop_all() {
	[ -n "$sim" ] && kill $sim 2>/dev/null
	[ ${#mine[@]} -gt 0 ] && kill "${mine[@]}" 2>/dev/null
	for _ in $(seq 1 30); do pgrep -f "^px4_mapping_gazebo|^/bin/bash \./run_sitl.sh" > /dev/null || break; sleep 1; done
}
trap stop_all EXIT

python3 -u scripts/ground_map.py "out/survey/$tag" --udp 14600 $([ "$VIEW" != 0 ] && echo --rviz) --world survey_site \
	> run/ground_map.log 2>&1 &
mine+=($!)
# RViz is left open at the end (not in mine), showing the last map; close it by hand.
[ "$VIEW" != 0 ] && setsid rviz2 -d rviz/ground.rviz > run/rviz_ground.log 2>&1 &

rm -f run/run_sitl.out
HEADLESS=$HEADLESS NO_RVIZ=1 RADARS=aop GPS=${GPS:-rtk} RADAR_LOG=1 PX4_DAEMON=1 QUIET_TIME=5 PX4_GZ_WORLD=survey_site \
	RADAR_ARGS="--frames-udp 14610 $SURFACE $RADAR_ARGS" setsid ./run_sitl.sh > run/run_sitl.out 2>&1 &
mine+=($!)
for _ in $(seq 1 120); do grep -q "PX4 SITL ready" run/run_sitl.out 2>/dev/null && break; sleep 2; done
grep -q "PX4 SITL ready" run/run_sitl.out || { echo "the simulation did not start (run/run_sitl.out)"; exit 1; }
sim=$(cat run/pids 2>/dev/null)

python3 -u scripts/onboard_live.py --frames 14610 --to 14600 --kbps "$kbps" > run/onboard_live.log 2>&1 &
onboard=$!
mine+=($onboard)
sleep 25
echo "flying the survey ($(date +%H:%M))"
python3 scripts/survey_mission.py --tag "$tag" | grep -E "^(waypoint [0-9]+/[0-9]+ reached|goal|closest|result|PX4 log|radar recording)"

# The on-board mapper ends 20 s after the last frame, the ground map 15 s after the last datagram: stop the
# simulation, then let both report.
[ -n "$sim" ] && kill $sim 2>/dev/null
wait "$onboard"
tail -2 run/onboard_live.log
sleep 20
tail -25 run/ground_map.log
if [ "$VIEW" != 0 ]; then echo "RViz stays open with the last map; close it when done"; fi
