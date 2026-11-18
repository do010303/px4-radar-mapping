#!/usr/bin/env bash
# Run the radar bench world and print what the emulated radar reports.
#
#   ./run_radar_test.sh            headless, one frame, table + out/radar_test.png
#   ./run_radar_test.sh --view     live: Gazebo GUI with the radar's coverage and
#                                  detections, plus RViz (source ROS 2 first)
#   ./run_radar_test.sh --gui      open the Gazebo GUI on the bench world instead
#   GZ_POINTS=1 ./run_radar_test.sh --view   detections in the Gazebo GUI too
#   RADAR_WORLD=<name> ...         another bench world (worlds/<name>.sdf)
#   GPU=igpu ...                   render on the integrated GPU (default: NVIDIA, if any)
#
# Waits until no other simulation runs (see sim_guard.sh). Gazebo runs in its
# own GZ_PARTITION, and only the server started here is stopped.

set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$here")"

export GZ_PARTITION="${GZ_PARTITION:-px4_mapping_radar}"
export GZ_SIM_RESOURCE_PATH="$root/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

radar_world="${RADAR_WORLD:-radar_test}"
world="$root/worlds/$radar_world.sdf"

BUILD="$(cd "$root/../.." && pwd)/build/px4_sitl_default"   # only for sim_guard.sh: it looks at every px4_sitl_* build
RUN_DIR="$root/run"
. "$here/sim_guard.sh"
wait_for_free_sim || exit 1

. "$here/gpu.sh"
use_nvidia_gpu

if [[ "${1:-}" == "--gui" ]]; then
	exec ruby "$here/gz_launch.rb" -r "$world"
fi

mkdir -p "$root/out"
log="$root/out/gz_server.log"

ruby "$here/gz_launch.rb" -s -r --headless-rendering "$world" >"$log" 2>&1 &
gz_pid=$!
trap 'kill "$gz_pid" 2>/dev/null || true; wait "$gz_pid" 2>/dev/null || true' EXIT

if [[ "${1:-}" == "--view" ]]; then
	cat "${HOME}/.gz/sim/8/gui.config" "$root/gui/radar_plugins.config" > "$root/out/gui.config"
	ruby "$here/gz_launch.rb" -g --gui-config "$root/out/gui.config" >/dev/null 2>&1 &
	points=()

	if [[ -n "${GZ_POINTS:-}" ]]; then
		# The detection markers go to the GUI, so it has to be listening first.
		until gz service -l 2>/dev/null | grep -q "^/marker_array$"; do sleep 1; done
		points=(--gz-points)
	fi

	python3 "$here/radar_viz.py" --world "$radar_world" --model radar_stand "${points[@]}" "${@:2}" &
	trap 'kill "$gz_pid" $(jobs -p) 2>/dev/null || true; wait 2>/dev/null || true' EXIT
	rviz2 -d "$root/rviz/radar.rviz"
	exit 0
fi

python3 "$here/radar_probe.py" --world "$radar_world" --plot "$root/out/radar_test.png" "$@" || {
	echo "--- gz server log: $log" >&2
	tail -n 20 "$log" >&2
	exit 1
}
