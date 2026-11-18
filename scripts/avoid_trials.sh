#!/bin/bash
# Repeat the avoidance flight to get a success rate: the headless simulation is restarted before each
# run (only the processes run_sitl.sh started, by PID), and every run keeps its own log and CSV.
#
#   scripts/avoid_trials.sh [runs] [world] [speed] [first]     default: 5 power_line 5 1
#   RADARS=6 scripts/avoid_trials.sh ...                       the 3x3 layout (default RADARS=3)
#   AVOID_ARGS="--mode brake" scripts/avoid_trials.sh ...      extra avoid_node.py options
#   GUI=1 scripts/avoid_trials.sh ...                          with the Gazebo GUI and RViz, to watch
#   NICE=0 scripts/avoid_trials.sh ...                         the simulation at normal priority (default 5)
#
# Writes out/trials/<world>_<speed>_<n>.{csv,txt} and one line per run to out/trials/summary.txt.

here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$here"
runs=${1:-5} world=${2:-power_line} speed=${3:-5} first=${4:-1}
mkdir -p out/trials

stop_sim() {
	[ -s run/pids ] && kill $(cat run/pids) 2>/dev/null
	# Until every earlier run_sitl.sh, and its exit watcher, is gone: the watcher stops the PIDs in
	# run/pids, which by then would be those of the next run.
	for _ in $(seq 1 30); do pgrep -f "^px4_mapping_gazebo|^/bin/bash \./run_sitl.sh" > /dev/null || break; sleep 1; done
	sleep 2
}

start_sim() {	# 0 once PX4 is ready; 1 when it is not within 120 s (a start that failed: gz_bridge did not spawn)
	stop_sim
	rm -f run/run_sitl.out
	HEADLESS=$([ -z "$GUI" ] && echo 1) RADARS=${RADARS:-3} PX4_DAEMON=1 QUIET_TIME=5 PX4_GZ_WORLD=$world nice -n "${NICE:-5}" setsid ./run_sitl.sh > run/run_sitl.out 2>&1 &

	for _ in $(seq 1 60); do
		grep -q "PX4 SITL ready" run/run_sitl.out 2>/dev/null && return 0
		sleep 2
	done

	return 1
}

for i in $(seq "$first" $((first + runs - 1))); do
	if ! start_sim && ! start_sim; then		# one retry
		echo "$world ${speed} m/s #$i: no flight (the simulation did not start, see run/run_sitl.out)" | tee -a out/trials/summary.txt
		continue
	fi

	sleep 15
	tag="out/trials/${world}_${speed}_$i"
	rm -f out/avoid.csv
	(. ./env.sh && python3 -u scripts/avoid_node.py --world "$world" --goal 45 0 10 --speed "$speed" --timeout 80 $AVOID_ARGS) > "$tag.txt" 2>&1
	code=$?
	[ -f out/avoid.csv ] && cp out/avoid.csv "$tag.csv"
	end=$(grep -E "^(goal reached|held by|collision)" "$tag.txt" | head -1)
	# A flight that ran to --timeout (close-proximity tests: the goal lies on a wire) ends with its result only.
	[ -z "$end" ] && end=$(grep -E "^result:" "$tag.txt" | sed 's/^result: */timeout, /')
	closest=$(grep "^closest" "$tag.txt")
	# Whether the run tested the avoidance at all: the cloud against the world, the radar lag, the host's load.
	cloud=$([ -f "$tag.csv" ] && python3 scripts/cloud_check.py "$tag.csv" | sed 's/^[^:]*: //')
	echo "$world ${speed} m/s #$i exit $code: ${end:-no flight (see $tag.txt)}; ${closest:-}" | tee -a out/trials/summary.txt
	echo "    ${cloud:-no CSV}; load $(cut -d' ' -f1 /proc/loadavg)$(grep -q 'NOT VALID' "$tag.txt" && echo '; NOT VALID: radar lag')" \
		| tee -a out/trials/summary.txt
done

stop_sim
