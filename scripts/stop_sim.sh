#!/bin/bash
# Stop everything this example started: run_sitl.sh and what it launched (Gazebo, PX4, RViz, radar nodes),
# scripts/avoid_node.py and scripts/avoid_trials.sh. Other simulations on the machine (px4-auto-tuning,
# ArduPilot, ...) are left alone: only this example's PIDs (run/pids) and processes whose command line
# points into this directory or at its PX4 rootfs are stopped.
#
#   scripts/stop_sim.sh
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ours() {
	{
		cat "$here/run/pids" 2>/dev/null
		pgrep -f "^px4_mapping_gazebo"                                # Gazebo server and GUI (scripts/gz_launch.rb)
		pgrep -f -- "-w $here/run/rootfs"                             # PX4
		pgrep -f "^(/bin/)?bash $here/(run_sitl\.sh|scripts/avoid_trials\.sh)"
		# Relative paths only from this directory: px4-auto-tuning runs a ./run_sitl.sh of its own.
		for p in $(pgrep -f "^(/bin/)?bash (-c )?(\./)?(run_sitl\.sh|scripts/avoid_trials\.sh)"); do
			[ "$(readlink "/proc/$p/cwd")" = "$here" ] && echo "$p"
		done
		pgrep -f "^python3 (-[us] )*($here/)?scripts/(avoid_node|radar_array|radar_viz|us_d1_node|us_d1_viz|onboard_live|ground_map)\.py"
		pgrep -f "^rviz2 -d $here/rviz/"
	} | grep -vx "$$" | sort -un
}

pids=$(ours)

if [ -z "$pids" ]; then
	echo "nothing of px4-mapping is running"
	exit 0
fi

echo "stopping:"
ps -o pid=,args= -p $(echo $pids | tr ' ' ',') | cut -c1-110
kill $pids 2>/dev/null

for _ in $(seq 1 10); do
	[ -z "$(ours)" ] && break
	sleep 0.5
done

left=$(ours)
[ -n "$left" ] && kill -KILL $left 2>/dev/null
rm -f "$here/run/pids"
echo "done"
