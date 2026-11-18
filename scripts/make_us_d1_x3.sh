#!/usr/bin/env bash
# Build the three radar models of models/x500_us_d1 (left, centre, right) from profiles/us_d1.yaml,
# and models/radar_mmwave, whose profile the tools take their radar model defaults from.
#
# The three scan at RATE Hz (default 25), not the sensor's 100 Hz: scripts/us_d1_node.py turns
# every scan into a packet in Python, and three radars at 100 Hz need more than one CPU core
# (measured: 26 Hz each at 109 % CPU, the scans queuing up). PX4's collision prevention drops a
# sensor only after 0.5 s without data. RATE=100 on a faster machine.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for side in left center right; do
	python3 "$here/make_radar_model.py" "$here/../profiles/us_d1.yaml" --name "radar_us_d1_$side" --rate "${RATE:-25}"
done

python3 "$here/make_radar_model.py" "$here/../profiles/us_d1.yaml"
