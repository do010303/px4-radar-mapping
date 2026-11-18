"""The map's coordinates in PX4's own local frame (docs/survey_mapping_results.md).

The mapping works in the site's world frame (ENU: x east, y north, z up, m, from the world's
spherical_coordinates; scoring against worlds/<world>.truth.yaml is done there). What it hands out, it can give
in PX4's local frame instead: NED (x north, y east, z down, m) from EKF2's origin, the point where the
vehicle's estimate started (its take-off point), as LOCAL_POSITION_NED, vehicle_local_position and the
setpoints use it. The origin is known in both: as a geodetic position (lat, lon, alt: absolute, from RTK) and
as an offset in the world frame (`origin`, ENU m: EstimatedPose.origin, onboard_live.LivePose.origin).

    ned = LocalNed(origin_enu, (lat, lon, alt))
    ned.points(world_xyz)         # (n, 3) world ENU -> local NED
    ned.object(o)                 # a survey_objects object (pole, wire, box) -> in local NED
"""

import numpy as np

ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])   # its own inverse
NAME = "px4_local_ned"


class LocalNed:
    def __init__(self, origin_enu, geo=None):
        self.origin = np.asarray(origin_enu, float).reshape(3)
        self.geo = None if geo is None else tuple(float(v) for v in geo)

    def points(self, xyz):
        """World ENU points (n, 3) (or one point) -> local NED."""
        xyz = np.asarray(xyz, float)
        return (xyz - self.origin) @ ENU_TO_NED.T

    def world(self, ned):
        """Local NED points -> world ENU."""
        return np.asarray(ned, float) @ ENU_TO_NED.T + self.origin

    def down(self, z_world):
        """A world height (z up) -> its D in the local frame."""
        return float(-(z_world - self.origin[2]))

    def object(self, o):
        """A survey_objects object in the world frame -> the same in local NED. A pole keeps its base point
        `p` and height `h` (up, m); a box's size becomes (north, east, down) extents."""
        out = dict(o)
        r3 = lambda v: np.round(v, 3).tolist()  # noqa: E731
        if o["type"] == "pole":
            out["p"] = r3(self.points(o["p"]))
        elif o["type"] == "wire":
            out["p0"], out["p1"] = r3(self.points(o["p0"])), r3(self.points(o["p1"]))
        else:
            out["c"] = r3(self.points(o["c"]))
            s = o["size"]
            out["size"] = r3([s[1], s[0], s[2]])
        return out

    def truth(self, truth):
        """worlds/<world>.truth.yaml's objects (pole, wire, box) in local NED, for plots."""
        out = {}
        for name, t in truth.items():
            t = dict(t)
            if t["type"] == "pole":
                t["p"] = self.points(t["p"]).tolist()
            elif t["type"] == "wire":
                t["p0"], t["p1"] = self.points(t["p0"]).tolist(), self.points(t["p1"]).tolist()
            else:
                t["c"] = self.points(t["c"]).tolist()
                t["size"] = [t["size"][1], t["size"][0], t["size"][2]]
            out[name] = t
        return out

    def header(self):
        """What a file of coordinates in this frame says about it."""
        h = {"frame": NAME, "axes": "x north, y east, z down (m), from EKF2's origin (PX4 local position)",
             "origin_in_world_enu": np.round(self.origin, 3).tolist()}
        if self.geo is not None:
            h["origin_lat"], h["origin_lon"], h["origin_alt"] = round(self.geo[0], 8), round(self.geo[1], 8), \
                round(self.geo[2], 3)
        return h
