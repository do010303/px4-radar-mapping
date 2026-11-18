#!/bin/bash
# Sensitivity tests of the radar mapping (docs/survey_mapping_plan.md): the survey flight over
# worlds/survey_site, once per radar variant, each with its own headless simulation (GPS=rtk, RADARS=aop,
# recorded), then scripts/survey_compare.py over all of them and the baseline flight.
#
#   scripts/survey_sensitivity.sh                  all variants
#   scripts/survey_sensitivity.sh angle2 worst     only these
#
# Starts and stops only the processes run_sitl.sh started (run/pids, by PID); waits for other simulations
# as run_sitl.sh does. Flights go to out/survey/sens_<variant>.*; the table to out/survey/sensitivity.md.

here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$here"

declare -A VARIANTS=(
	[angle2]="--angle-scale 2"
	[thin-5]="--thin-db -5"
	[ghost5]="--angle-outlier 0.05"
	[surface+10]="--surface-db 10"
	[points3]="--points-scale 3"
	[bright]="--surface-db 10 --points-scale 3"
	[worst]="--angle-scale 2 --thin-db -5 --angle-outlier 0.05"
)
ORDER="angle2 thin-5 ghost5 surface+10 points3 bright worst"
names=${*:-$ORDER}

stop_sim() {
	[ -s run/pids ] && kill $(cat run/pids) 2>/dev/null
	[ -n "$sitl" ] && kill "$sitl" 2>/dev/null
	for _ in $(seq 1 30); do pgrep -f "^px4_mapping_gazebo|^/bin/bash \./run_sitl.sh" > /dev/null || break; sleep 1; done
	sleep 3
}

for name in $names; do
	args=${VARIANTS[$name]}
	[ -z "$args" ] && { echo "unknown variant $name"; continue; }
	echo "== $name ($args), $(date +%H:%M), load $(cut -d' ' -f1 /proc/loadavg)"
	rm -f run/run_sitl.out
	HEADLESS=1 RADARS=aop GPS=rtk RADAR_LOG=1 PX4_DAEMON=1 QUIET_TIME=5 PX4_GZ_WORLD=survey_site RADAR_ARGS="$args" \
		setsid ./run_sitl.sh > run/run_sitl.out 2>&1 &
	sitl=$!
	ready=
	for _ in $(seq 1 90); do grep -q "PX4 SITL ready" run/run_sitl.out 2>/dev/null && { ready=1; break; }; sleep 2; done
	if [ -z "$ready" ]; then
		echo "   the simulation did not start (run/run_sitl.out)"; stop_sim; continue
	fi
	sitl=$(pgrep -f "^/bin/bash \./run_sitl.sh" | head -1)
	sleep 25
	timeout 900 python3 scripts/survey_mission.py --tag "sens_$name" > /dev/null 2>&1
	grep -E "^(closest|result)" "out/survey/sens_$name.txt" | sed 's/^/   /'
	stop_sim
done

tags="out/survey/rtk_3"
for name in $ORDER; do [ -d "out/survey/sens_$name.radar" ] && tags="$tags out/survey/sens_$name"; done
python3 scripts/survey_compare.py $tags | tee out/survey/sensitivity.md
echo "ALL DONE"
