#!/bin/bash
# PX4 SITL with the radar-carrying x500 (models/x500_radar) in worlds/radar_hover.
#
#   ./run_sitl.sh                 Gazebo GUI with radar coverage and detections, plus RViz
#   NO_RVIZ=1 ./run_sitl.sh       without RViz
#   HEADLESS=1 ./run_sitl.sh      no Gazebo GUI and no RViz (e.g. for scripts/hover_test.py)
#   PX4_GZ_WORLD=... ./run_sitl.sh   another world from ./worlds
#   GZ_POINTS=1 ./run_sitl.sh     detections in the Gazebo GUI too (slower; RViz shows them)
#   RADAR_ARGS="--doppler-gate 0" ./run_sitl.sh   extra scripts/radar_array.py options (with RADARS)
#   GPS=rtk ./run_sitl.sh         an RTK fix (2 cm) that EKF2 trusts as such, for mapping (docs/survey_mapping_plan.md);
#                                 GPS=std: an ordinary receiver (~1 m); unset: PX4's simulation defaults
#   RADAR_LOG=1 ./run_sitl.sh     also record every radar frame as the hardware gives it, to run/radar_log/<time>/
#                                 (scripts/radar_log.py; scripts/survey_mission.py links it to its flight)
#   RADAR_FAST_PB=0 ./run_sitl.sh  radar_array.py with the pure-Python protobuf (slow; see RADAR_PY below)
#   GPU=igpu ./run_sitl.sh        render on the integrated GPU (default: NVIDIA, if any)
#   US_D1=3 ./run_sitl.sh         three of them on models/x500_us_d1 (left, ahead, right: ~130 deg);
#                                 run scripts/make_us_d1_x3.sh first
#   RADARS=6 ./run_sitl.sh        six: that ring plus an upright one, 3 x 3 (models/x500_radar6): 3D, 62 % of all directions
#   RADARS=aop ./run_sitl.sh      the paper's mix: 1 ISK ahead + 5 AOP (models/x500_radar_aop): 91 % of all directions
#   RADARS=3 ./run_sitl.sh        three IWR6843ISK radars, 360 deg (models/x500_radar3, world power_line;
#                                 run scripts/make_isk_x3.sh first); scripts/radar_array.py publishes
#                                 the merged cloud, scripts/avoid_node.py flies on it
#   US_D1=1 ./run_sitl.sh         the Ainstein US-D1 (python3 scripts/make_radar_model.py
#                                 profiles/us_d1.yaml first): its UART packets go to a pty and
#                                 the us_d1_radar driver of px4_module/ reads them. Needs
#                                 scripts/build_px4.sh; the shared px4_sitl_default is left alone.
#
# From a terminal PX4 runs in the foreground with its `pxh>` console; without
# one (or with PX4_DAEMON=1) it runs as a daemon and logs to run/px4.log.
# Then fly it with scripts/hover_test.py, or by hand at the pxh> prompt
# (`commander takeoff`, `commander land`).
#
# Other simulations on this machine come first (in particular the ArduPilot
# rover work under ~/UAV, which starts and stops Gazebo on its own): while one
# runs, this script waits before starting, and if one starts while this
# simulation runs, this one stops to make room. WAIT_TIMEOUT=<s> gives up
# waiting after that long (default: wait indefinitely).
#
# Gazebo runs in its own transport partition (GZ_PARTITION=px4_mapping). Use
# the same partition for `gz topic` etc.: `source env.sh`.

set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$(cd "$HERE/../.." && pwd)}"
# This example's own build (scripts/build_px4.sh) if there is one; the shared default build otherwise.
if [ -z "$PX4_BUILD" ]; then
	PX4_BUILD="$PX4_DIR/build/px4_sitl_mapping"
	[ -x "$PX4_BUILD/bin/px4" ] || PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
fi

BUILD="$PX4_BUILD"
RUN_DIR="${RUN_DIR:-$HERE/run}"

. "$HERE/env.sh"

mkdir -p "$RUN_DIR"

if [ ! -x "$BUILD/bin/px4" ]; then
	echo "PX4 SITL is not built: run 'make px4_sitl' in $PX4_DIR first" >&2
	exit 1
fi

. "$HERE/scripts/sim_guard.sh"
wait_for_free_sim || exit 1

. "$HERE/scripts/gpu.sh"
use_nvidia_gpu

export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4001}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500_radar}"
radar_suffix=$RADARS
[ "$RADARS" = aop ] && radar_suffix=_aop		# models/x500_radar_aop
[ -n "$RADARS" ] && export PX4_SIM_MODEL="${PX4_SIM_MODEL/gz_x500_radar/gz_x500_radar$radar_suffix}"
[ -n "$RADARS" ] && export PX4_GZ_WORLD="${PX4_GZ_WORLD:-power_line}"
[ "$US_D1" = 3 ] && export PX4_SIM_MODEL="${PX4_SIM_MODEL/gz_x500_radar/gz_x500_us_d1}"	# PX4_SIM_MODEL=gz_x500_us_d1_up15 for the radars tilted 15 deg up
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-radar_hover}"
WORLD_FILE="$HERE/worlds/$PX4_GZ_WORLD.sdf"
MODEL_NAME="${PX4_SIM_MODEL#gz_}_0"

# The Gazebo server and GUI are started here (PX4 then attaches to the running
# world), through scripts/gz_launch.rb: same as `gz sim`, but under a process
# title that a `pkill -f 'gz sim'` from other simulation tooling does not hit.
GZ_LAUNCH="ruby $HERE/scripts/gz_launch.rb"
GUI_WANTED=
[ -z "$HEADLESS" ] && GUI_WANTED=1
export HEADLESS=1

# No RC and no ground station in this setup: do not fail-safe on their absence.
export PX4_PARAM_NAV_DLL_ACT=0
export PX4_PARAM_NAV_RCL_ACT=0
export PX4_PARAM_COM_RC_IN_MODE=4
export PX4_PARAM_MIS_TAKEOFF_ALT="${TAKEOFF_ALT:-2.5}"

# Collision prevention (Position mode) needs a manual control source and CP_DIST >= 0:
# CP_DIST=5 ./run_sitl.sh turns it on, with the stick input coming over MAVLink
# (scripts/cp_test.py). Without it the vehicle is flown by commands only.
if [ -n "$CP_DIST" ]; then
	export PX4_PARAM_CP_DIST="$CP_DIST"
	export PX4_PARAM_COM_RC_IN_MODE=1
fi

# The GPS for mapping. The simulated receiver's noise (SIM_GZ_GPS_PNOIS, horizontal sigma about 0.027 x it)
# is 2 cm by default already, but it reports 0.9 m accuracy and EKF2 weighs it as 0.5 m, so the estimate
# wanders by up to a metre; GPS=rtk reports and weighs it as the 2 cm it is. And the simulated fix comes
# without delay: the default SENS_GPS0_DELAY of 0.11 s put the estimate 0.11 s ahead of the truth (0.3 m at
# 4 m/s, and a heading error of several degrees; scripts/pose_check.py on out/survey/rtk_1.ulg).
case "$GPS" in
	rtk)
		export PX4_PARAM_SIM_GZ_GPS_PNOIS=0.8 PX4_PARAM_SIM_GZ_GPS_EPH=0.02 PX4_PARAM_SIM_GZ_GPS_EPV=0.03
		export PX4_PARAM_EKF2_GPS_P_NOISE=0.02 PX4_PARAM_EKF2_GPS_V_NOISE=0.05 PX4_PARAM_SENS_GPS0_DELAY=0
		;;
	std)
		# About 1 m, wandering: faster than the pre-flight drift check allows (0.1 m/s), so that is relaxed.
		export PX4_PARAM_SIM_GZ_GPS_PNOIS=40 PX4_PARAM_SIM_GZ_GPS_EPH=1.5 PX4_PARAM_SIM_GZ_GPS_EPV=2.5
		export PX4_PARAM_SENS_GPS0_DELAY=0 PX4_PARAM_EKF2_REQ_HDRIFT=0.5 PX4_PARAM_EKF2_REQ_VDRIFT=1.0
		;;
	"") ;;
	*) echo "GPS=$GPS: rtk, std or unset" >&2; exit 1 ;;
esac

# Every background process this script starts is recorded here and stopped on
# exit, by PID; nothing else on the machine is touched.
# run/pids is the latest run's, for scripts/stop_sim.sh and scripts/survey_live.sh; this script stops what is
# in its own list (run/pids.<its PID>), so that a run starting as this one ends cannot have its processes
# stopped by this one's cleanup.
PIDS="$RUN_DIR/pids"
OWN="$RUN_DIR/pids.$$"
: > "$PIDS"
: > "$OWN"

track() {
	echo "$1" >> "$PIDS"
	echo "$1" >> "$OWN"
}

cleanup() {
	set +e
	local pids
	pids=$(cat "$OWN" 2>/dev/null)
	[ -n "$pids" ] && kill $pids 2>/dev/null
	# Give them 3 s to exit; the Gazebo GUI does not always quit on SIGTERM.
	for _ in 1 2 3 4 5 6; do
		local alive=
		for p in $pids; do kill -0 "$p" 2>/dev/null && alive=1; done
		[ -z "$alive" ] && break
		sleep 0.5
	done
	[ -n "$pids" ] && kill -KILL $pids 2>/dev/null
	rm -f "$OWN"
	# The shared list only if it is still this run's.
	[ "$(cat "$PIDS" 2>/dev/null)" = "$pids" ] && rm -f "$PIDS"
}
trap cleanup EXIT

# If this script dies without running cleanup (its terminal killed, say), the
# processes it started would run on with nobody to stop them; this watcher,
# which ignores the terminal's signals, stops them then.
(
	trap '' HUP INT
	while kill -0 $$ 2>/dev/null; do sleep 2; done
	pids=$(cat "$OWN" 2>/dev/null)
	[ -n "$pids" ] || exit 0
	kill $pids 2>/dev/null
	sleep 3
	kill -KILL $pids 2>/dev/null
) &

# Returns 0 as soon as another simulation starts, 1 once PX4 (pid $1) has exited.
watch_for_other_sims() {
	while kill -0 "$1" 2>/dev/null; do
		if [ -n "$(other_sims)" ]; then
			echo -e "\nAnother simulation started; stopping this one to make room:" >&2
			other_sims | xargs -r ps -o pid=,args= -p | cut -c1-120 >&2
			return 0
		fi

		sleep 3
	done

	return 1
}

# Own rootfs: parameters start from firmware defaults on every start (the
# previous session's are kept as *.bson.prev).
ROOTFS="$RUN_DIR/rootfs"
mkdir -p "$ROOTFS"

for f in "$ROOTFS/parameters.bson" "$ROOTFS/parameters_backup.bson"; do
	[ -f "$f" ] && mv -f "$f" "$f.prev"
done

# One model directory with PX4's models plus ours, because PX4 spawns from a
# single PX4_GZ_MODELS path; worlds come from ./worlds.
rm -rf "$RUN_DIR/models" && mkdir -p "$RUN_DIR/models"
ln -s "$PX4_DIR/Tools/simulation/gz/models"/* "$RUN_DIR/models/"
ln -sf "$HERE"/models/* "$RUN_DIR/models/"
cp "$BUILD/rootfs/gz_env.sh" "$ROOTFS/gz_env.sh"
cat >> "$ROOTFS/gz_env.sh" <<EOT
export PX4_GZ_MODELS="$RUN_DIR/models"
export PX4_GZ_WORLDS="$HERE/worlds"
export GZ_SIM_RESOURCE_PATH="\$GZ_SIM_RESOURCE_PATH:$RUN_DIR/models:$HERE/worlds"
EOT
. "$ROOTFS/gz_env.sh"

cd "$ROOTFS"

if gz topic -l 2>/dev/null | grep -q "^/world/.*/clock$"; then
	echo "A Gazebo world is already running in partition $GZ_PARTITION; stop it first." >&2
	exit 1
fi

$GZ_LAUNCH --verbose=1 -r -s "$WORLD_FILE" > "$RUN_DIR/gz_server.log" 2>&1 &
GZ_PID=$!
track $GZ_PID

until gz service -i --service "/world/$PX4_GZ_WORLD/scene/info" 2>/dev/null | grep -q "Service providers"; do
	kill -0 $GZ_PID 2>/dev/null || { tail "$RUN_DIR/gz_server.log"; exit 1; }
	sleep 1
done

# The US-D1's serial line: the radar node writes the sensor's packets to a pty and
# the PX4 driver reads them, as on the real wiring.
start_us_d1() {
	# The driver keeps the device path in 20 bytes, so it is given the short, relative name
	# of a link in PX4's working directory (the rootfs).
	local tty="$ROOTFS/us_d1_0" sensors= rotations="${US_D1_ROTATION:-0}"

	if [ "$US_D1" = 3 ]; then
		# Left, ahead, right: PX4 rotations YAW_315, FORWARD_FACING, YAW_45.
		sensors="--sensors left,center,right"
		rotations="7 0 1"
	fi

	python3 "$HERE/scripts/us_d1_node.py" --world "$PX4_GZ_WORLD" --model "$MODEL_NAME" --tty "$tty" $sensors \
		> "$RUN_DIR/us_d1_node.log" 2>&1 &
	track $!

	(
		# PX4 must be up before its driver can be started.
		: > "$RUN_DIR/us_d1_driver.log"
		n=0
		for rotation in $rotations; do
			for _ in $(seq 1 60); do
				[ -e "$ROOTFS/us_d1_$n" ] && "$BUILD/bin/px4-us_d1_radar" start -d "us_d1_$n" -R "$rotation" \
					>> "$RUN_DIR/us_d1_driver.log" 2>&1 && break
				sleep 2
			done
			n=$((n + 1))
		done
	) &
	track $!
}

# radar_array.py decodes 60 laser scans a second. The gz-msgs Python code is generated for protobuf 3, so with
# the protobuf 6 of the user's site it needs the pure-Python backend (scripts/radar_model.py): 4 ms a scan,
# a quarter of a core, and with the Gazebo GUI on as well the radars fell behind (the avoidance then holds).
# The system's protobuf 3.12 runs it in C++ (0.2 ms): without the user site, if that works. RADAR_FAST_PB=0: as before.
RADAR_PY=python3
if [ "$RADAR_FAST_PB" != 0 ] && PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp python3 -s -c \
	"import sys; sys.path.insert(0, '$HERE/scripts'); import radar_array
from google.protobuf.internal import api_implementation as a; assert a.Type() == 'cpp'" > /dev/null 2>&1; then
	RADAR_PY="env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp python3 -s"
fi

start_viewers() {
	# Wait until the vehicle exists in Gazebo, then attach the viewers to it.
	local scan=/radar/scan
	[ "$US_D1" = 3 ] && scan=/radar/scan_us_d1_center
	[ -n "$RADARS" ] && scan=/radar/scan_isk_0
	[ "$RADARS" = aop ] && scan=/radar/scan_isk_front
	until gz topic -l 2>/dev/null | grep -q "^$scan$"; do sleep 1; done

	if [ -n "$GUI_WANTED" ]; then
		cat "$HOME/.gz/sim/8/gui.config" "$HERE/gui/radar_plugins.config" > "$RUN_DIR/gui.config"
		$GZ_LAUNCH -g --gui-config "$RUN_DIR/gui.config" > "$RUN_DIR/gz_gui.log" 2>&1 &
		track $!

		if [ -n "$GZ_POINTS" ]; then
			# The detection markers go to the GUI, so it has to be listening first.
			until gz service -l 2>/dev/null | grep -q "^/marker_array$"; do sleep 1; done
			sleep 2
		fi
	fi

	if [ -n "$RADARS" ]; then
		local log_args=
		[ -n "$RADAR_LOG" ] && log_args="--log $RUN_DIR/radar_log/$(date +%Y%m%d_%H%M%S)"
		$RADAR_PY "$HERE/scripts/radar_array.py" --world "$PX4_GZ_WORLD" --model "$MODEL_NAME" --layout "$RADARS" $log_args $RADAR_ARGS > "$RUN_DIR/radar_array.log" 2>&1 &
		track $!
	elif [ "$US_D1" = 3 ]; then
		# The three radars are shown as PX4 sees them (its MAVLink stream), not from the scans.
		local tilt=0
		case "$PX4_SIM_MODEL" in *up15*) tilt=15;; esac
		python3 "$HERE/scripts/us_d1_viz.py" --tilt-deg "$tilt" > "$RUN_DIR/us_d1_viz.log" 2>&1 &
		track $!
	else
		python3 "$HERE/scripts/radar_viz.py" --world "$PX4_GZ_WORLD" --model "$MODEL_NAME" \
			$([ -n "$GUI_WANTED" ] && [ -n "$GZ_POINTS" ] && echo --gz-points) > "$RUN_DIR/radar_viz.log" 2>&1 &
		track $!
	fi

	[ -z "$GUI_WANTED" ] && return 0

	# Camera behind and above the vehicle, so the radar's coverage is in view.
	(sleep 5; gz topic -t /gui/track -m gz.msgs.CameraTrack -p "track_mode: FOLLOW, \
		follow_target: {name: '$MODEL_NAME'}, follow_offset: {x: -6.0, y: -3.0, z: 4.0}, \
		follow_pgain: 1.0, track_pgain: 1.0") > /dev/null 2>&1 &

	if [ -z "$NO_RVIZ" ]; then
		local rviz_config=radar.rviz
		[ "$US_D1" = 3 ] && rviz_config=us_d1.rviz
		[ -n "$RADARS" ] && rviz_config=radar3.rviz
		rviz2 -d "$HERE/rviz/$rviz_config" > "$RUN_DIR/rviz.log" 2>&1 &
		track $!
	fi
}

if [ -n "$US_D1" ] && [ ! -x "$BUILD/bin/px4-us_d1_radar" ]; then
	echo "US_D1=1 needs the us_d1_radar driver: run scripts/build_px4.sh" >&2
	exit 1
fi

if [ -t 0 ] && [ -t 1 ] && [ -z "$PX4_DAEMON" ]; then
	start_viewers &
	track $!
	[ -n "$US_D1" ] && start_us_d1

	# PX4 holds the terminal, so its PID is looked up once it runs; then make
	# room as soon as another simulation starts.
	(
		until PX4_PID=$(pgrep -f -x "$BUILD/bin/px4 $BUILD/etc -w $ROOTFS"); do sleep 1; done
		watch_for_other_sims "$PX4_PID" && kill "$PX4_PID"
	) &
	track $!

	set +e
	"$BUILD/bin/px4" "$BUILD/etc" -w "$ROOTFS"
	exit $?
fi

"$BUILD/bin/px4" -d "$BUILD/etc" -w "$ROOTFS" > "$RUN_DIR/px4.log" 2>&1 &
PX4_PID=$!
track $PX4_PID

until grep -q "Startup script returned successfully" "$RUN_DIR/px4.log"; do
	kill -0 $PX4_PID 2>/dev/null || { cat "$RUN_DIR/px4.log"; exit 1; }
	sleep 1
done

start_viewers
[ -n "$US_D1" ] && start_us_d1
echo "PX4 SITL ready (world: $PX4_GZ_WORLD, model: $MODEL_NAME, log: $RUN_DIR/px4.log)"
watch_for_other_sims $PX4_PID || true
