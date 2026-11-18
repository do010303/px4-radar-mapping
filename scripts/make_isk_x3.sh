#!/usr/bin/env bash
# Build the radar models of models/x500_radar3 (a level ring of 3 ISK), models/x500_radar6 (3 x 3 ISK) and
# models/x500_radar_aop (1 ISK + 5 AOP), the vehicle SDFs, and models/radar_mmwave, whose profile the tools take
# their radar model defaults from. All from profiles/iwr6843isk.yaml; mounts in scripts/radar_layouts.py.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
profile="$here/../profiles/iwr6843isk.yaml"

# Each mount from its own profile: IWR6843AOP for aop_* (x500_radar_aop), IWR6843ISK otherwise.
python3 -c "import sys; sys.path.insert(0, '$here'); import radar_layouts as r; print('\n'.join(f'{n} {r.profile_path(n)}' for l in r.LAYOUTS.values() for n, _, _ in l))" |
while read -r name mount_profile; do
	python3 "$here/make_radar_model.py" "$mount_profile" --name "radar_$name"
done

python3 "$here/make_radar_model.py" "$profile"
python3 "$here/radar_layouts.py" "$here/../models"
