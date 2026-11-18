#!/usr/bin/env python3
"""Whether a flight's radar cloud matched the world: d_cloud (the nearest radar point) against the true nearest
obstacle, over the rows where that is within 6 m; and the radar lag (column radar_lag) when logged.

A valid run (the radar processing keeping up) has the median within about -0.1 m and nothing missed; under a
load of 50-60 the clouds lagged and the median went to -1 m, with a fifth of the obstacles missed (runs
#921-926), for the old pipeline as for the new.

    python3 scripts/cloud_check.py out/trials/power_line_course_5_931.csv [...]
"""

import csv
import sys

import numpy as np


def check(path):
    rows = list(csv.DictReader(open(path)))
    d = np.array([float(r["d_cloud"]) for r in rows])
    true = np.array([min(float(r["d_wire"]), float(r["d_obstacle"])) for r in rows])
    near = true < 6.0

    if not near.any():
        return "cloud: nothing within 6 m"

    err = d[near] - true[near]
    text = (f"cloud - true {np.median(err):+.2f} m median, {np.percentile(err, 5):+.2f} m 5 %, "
            f"missed {np.mean(err > 2.0) * 100:.0f} %")
    lag = np.array([float(r.get("radar_lag") or "nan") for r in rows])
    lag = lag[np.isfinite(lag)]

    if len(lag):
        text += f"; radar lag {np.percentile(lag, 95):.2f} s 95 %"

    return text


if __name__ == "__main__":
    for path in sys.argv[1:]:
        print(f"{path}: {check(path)}")
