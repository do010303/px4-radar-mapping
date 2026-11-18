#!/usr/bin/env bash
# Build PX4 SITL with this example's own modules (px4_module/) into build/px4_sitl_mapping.
#
#   ./scripts/build_px4.sh            configure if needed, then build (low priority)
#   JOBS=4 ./scripts/build_px4.sh     compile jobs (default 6)
#
# The other examples run build/px4_sitl_default; this one never touches it. It calls
# cmake directly instead of `make px4_sitl_*`, because the Makefile records the last
# target in build/.last_target and cleans the NuttX submodules when it changes.
# Nothing is written outside build/px4_sitl_mapping, and nothing in PX4's src/ or boards/.

set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(dirname "$here")"
px4="$(cd "$root/../.." && pwd)"
build="${PX4_BUILD:-$px4/build/px4_sitl_mapping}"

case "$build" in
	"$px4"/build/px4_sitl_default | "$px4"/build/px4_sitl_default/*)
		echo "refusing to build into the shared px4_sitl_default" >&2
		exit 1;;
esac

if [ ! -f "$build/CMakeCache.txt" ]; then
	mkdir -p "$build"
	(cd "$build" && nice -n 19 cmake "$px4" -GNinja -DCONFIG=px4_sitl_default \
		-DEXTERNAL_MODULES_LOCATION:STRING="$root/px4_module")
fi

nice -n 19 ionice -c3 cmake --build "$build" -- -j"${JOBS:-6}"
ls -l "$build/bin/px4" "$build/bin/px4-us_d1_radar"
