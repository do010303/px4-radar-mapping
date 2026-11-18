"""3D mmWave radar model: Gazebo ray grid in, sparse radar detections out.

Gazebo gives a dense, noise-free range for every ray of the radar_mmwave
sensor. A 3D mmWave radar instead reports a sparse set of detections (x, y, z,
radial velocity, SNR), so per frame:

  1. rays are grouped into resolution cells (range bin x azimuth x elevation);
  2. each cell gets an SNR from the radar equation: 1/R^4 range loss, antenna
     gain falling off with elevation (and with azimuth for a sector antenna),
     how much of the cell the surface fills, and how squarely the surface
     faces the radar;
  3. cells above the detection threshold become one point each, with range and
     angle error (elevation worse than azimuth). A radar with a handful of
     antennas resolves only a couple of directions per range bin, so each range
     bin keeps its strongest few cells, and the frame is capped overall;
  3b. a "beam" antenna takes its pattern from the half-power beamwidths of a
     datasheet (Gaussian main lobe, a sidelobe floor), and a profile with a link
     budget (eirp_dbm, ...) gets SNR from the radar equation of a surface patch
     instead of the reference SNR; a range-only sensor (output: range) reports
     just its nearest detection, on boresight, as it cannot tell where in its
     beam the return came from;
  4. radial velocity is the sensor's own velocity projected on the line of sight
     (the world is assumed static, the sensor's rotation is ignored).

Wires and poles thinner than the ray spacing are not seen by the ray grid; wire_detections()
gives them the radar cross section of a thin cylinder instead.

Not modelled: multipath ghosts, material-dependent reflectivity, moving targets.
"""

import os

# The distro's gz.msgs bindings predate the protobuf wheel pip may have
# installed; the pure-Python implementation still accepts them. Every script
# imports this module before any gz.msgs module, so this is the one place.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import numpy as np


PROFILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "radar_mmwave", "profile.yaml")

# Command line help for the model keys of a profile (profiles/*.yaml).
MODEL_HELP = {
    "antenna": "omni: gain falls off with elevation only; sector: also away from boresight; "
               "beam: Gaussian main lobe from --beam-az/--beam-el",
    "min_range": "ignore returns closer than this (m), e.g. the vehicle's own frame",
    "range_bin": "range resolution (m)",
    "angle_cell": "angular resolution cell (deg)",
    "snr_threshold": "detection threshold (dB)",
    "snr_sigma": "frame-to-frame SNR fluctuation (dB)",
    "specular": "how fast returns fade as a surface turns away: SNR += 10*specular*log10(cos incidence)",
    "sigma_range": "range error std-dev (m)",
    "sigma_az": "azimuth error std-dev (deg)",
    "sigma_el": "elevation error std-dev (deg)",
    "per_range_bin": "directions resolved within one range bin",
    "max_points": "detections per frame, strongest first",
}


INT_KEYS = ("per_range_bin", "max_points")

# Keys a profile may leave out: (help, default). None disables the feature.
OPTIONAL_HELP = {
    "snr_ref": ("SNR of a surface filling a whole cell, facing the radar on boresight, at --range-ref (dB); "
                "needed unless the profile has a link budget", None),
    "range_ref": ("range of --snr-ref (m)", 10.0),
    "beam_az": ("azimuth half-power beamwidth, full width (deg), antenna beam", 50.0),
    "beam_el": ("elevation half-power beamwidth, full width (deg), antenna beam", 30.0),
    "sidelobe_db": ("antenna beam: one-way gain floor below boresight (dB)", 25.0),
    "output": ("points: every detection; range: only the nearest, on boresight (single-beam sensor); "
               "altimeter: the one return the sensor reports, with its SNR, whether or not it is above the threshold",
               "points"),
    "pick": ("altimeter output: which return the sensor reports, the strongest or the nearest above the threshold",
             "strongest"),
    "eirp_dbm": ("link budget: radiated power incl. antenna gain (dBm); omit to use --snr-ref", None),
    "rx_gain_dbi": ("link budget: receive antenna gain (dBi)", 0.0),
    "wavelength_mm": ("link budget: wavelength (mm)", 12.4),
    "noise_dbm": ("link budget: noise power after integration (dBm)", -149.0),
    "loss_db": ("link budget: system loss (dB)", 6.0),
    "reflectivity_db": ("link budget: share of the power a surface scatters back to the radar (dB)", -10.0),
    "sigma_velocity": ("radial velocity noise, 1 sigma (m/s)", 0.0),
    "angle_outlier": ("share of detections placed at a wrong angle, up to two angle cells off (sidelobes, angle "
                      "ambiguity); their radial velocity stays that of the true direction", 0.0),
}


def load_profile(path=PROFILE):
    import yaml

    with open(path) as f:
        return yaml.safe_load(f)


def add_model_args(p, profile=PROFILE):
    """Radar model options, defaulting to the model section of the active profile.

    The active profile is the one models/radar_mmwave was built from
    (scripts/make_radar_model.py), so the model matches the Gazebo sensor.
    """
    model = load_profile(profile)["model"]

    for key, help_text in MODEL_HELP.items():
        value = model[key]
        option = "--" + key.replace("_", "-")

        if key == "antenna":
            p.add_argument(option, choices=("omni", "sector", "beam"), default=value, help=help_text)
        elif key in INT_KEYS:
            p.add_argument(option, type=int, default=int(value), help=help_text)
        else:
            p.add_argument(option, type=float, default=float(value), help=help_text)

    for key, (help_text, default) in OPTIONAL_HELP.items():
        value = model.get(key, default)
        option = "--" + key.replace("_", "-")

        if isinstance(default, str):
            choices = {"output": ("points", "range", "altimeter"), "pick": ("strongest", "nearest")}[key]
            p.add_argument(option, choices=choices, default=value, help=help_text)
        else:
            p.add_argument(option, type=float, default=None if value is None else float(value), help=help_text)


def scan_to_grid(msg):
    """Unpack a LaserScan into ranges[el, az] plus the angle axes in radians."""
    n_az = msg.count
    n_el = max(msg.vertical_count, 1)
    ranges = np.array(msg.ranges, dtype=float).reshape(n_el, n_az)
    ranges[~np.isfinite(ranges)] = np.nan
    az = msg.angle_min + msg.angle_step * np.arange(n_az)
    el = msg.vertical_angle_min + msg.vertical_angle_step * np.arange(n_el)
    return ranges, az, el


def incidence_cos(points, dirs):
    """|cos| of the angle between each ray and the surface it hit, from grid neighbours."""
    d_az = np.gradient(points, axis=1)
    d_el = np.gradient(points, axis=0)
    normal = np.cross(d_az, d_el)
    norm = np.linalg.norm(normal, axis=-1)

    with np.errstate(invalid="ignore", divide="ignore"):
        cos_inc = np.abs(np.sum(normal * dirs, axis=-1)) / norm

    # Edges and objects thinner than the ray spacing have no usable normal; they
    # scatter in all directions, so treat them as facing the radar.
    return np.where(np.isfinite(cos_inc), cos_inc, 1.0)


def two_way_beam_db(az, el, cfg):
    """Two-way gain (dB) of a beam antenna at (az, el) off boresight, rad.

    Gaussian main lobe: a pattern whose one-way power is down 3 dB at half the
    half-power beamwidth, i.e. 12 dB * (angle / beamwidth)^2 per axis, with a
    sidelobe floor. The radar transmits and receives with it, so it counts twice.
    """
    one_way = -12.0 * ((np.degrees(az) / cfg.beam_az) ** 2 + (np.degrees(el) / cfg.beam_el) ** 2)
    return 2.0 * np.maximum(one_way, -cfg.sidelobe_db)


def antenna_gain_db(a, e, cfg):
    """Two-way antenna gain (dB) at azimuth a, elevation e (rad) off boresight."""
    if cfg.antenna == "beam":
        return two_way_beam_db(a, e, cfg)

    # Two-way antenna pattern, cos^2 each way, floored to avoid log(0) at the edge.
    pattern = np.cos(e) * (np.cos(a) if cfg.antenna == "sector" else 1.0)
    return 10.0 * np.log10(np.maximum(pattern, 1e-3) ** 4)


def display_angles(profile, angles):
    """(az_min, az_max, el_min, el_max) of the field of view to draw, rad.

    The scan, unless display.az_deg / display.el_deg (half-angles) are given: a
    beam antenna is scanned wider than its half-power beam so the sidelobes
    are in the data, but the drawn field of view is the half-power one.
    """
    d = profile.get("display", {})
    az_half = d.get("az_deg")
    el_half = d.get("el_deg")
    az_min, az_max, el_min, el_max = angles

    if az_half is not None:
        az_min, az_max = -np.radians(az_half), np.radians(az_half)

    if el_half is not None:
        el_min, el_max = -np.radians(el_half), np.radians(el_half)

    return az_min, az_max, el_min, el_max


def detect(ranges, az, el, cfg, rng, velocity=(0.0, 0.0, 0.0), truth=False):
    """Return radar detections as an (N, 5) array: x, y, z, radial velocity, SNR dB.

    With truth, also return where each detection really came from: the (N, 3)
    centroid of the ray hits in its resolution cell, without the radar's errors.
    """
    az_g, el_g = np.meshgrid(az, el)
    dirs = np.stack((np.cos(el_g) * np.cos(az_g), np.cos(el_g) * np.sin(az_g), np.sin(el_g)), axis=-1)
    points = ranges[..., None] * dirs
    cos_inc = incidence_cos(points, dirs)

    hit = np.isfinite(ranges) & (ranges >= cfg.min_range)

    if not hit.any():
        return (np.empty((0, 5)), np.empty((0, 3))) if truth else np.empty((0, 5))

    cell = np.radians(cfg.angle_cell)
    key = np.stack((np.floor(ranges[hit] / cfg.range_bin),
                    np.floor((az_g[hit] - az[0]) / cell),
                    np.floor((el_g[hit] - el[0]) / cell)), axis=1).astype(np.int64)
    _, inverse, count = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.ravel()

    def cell_mean(values):
        return np.bincount(inverse, weights=values, minlength=len(count)) / count

    r = cell_mean(ranges[hit])
    a = cell_mean(az_g[hit])
    e = cell_mean(el_g[hit])
    facing = cell_mean(cos_inc[hit])
    centroid = np.column_stack([cell_mean(points[hit][:, i]) for i in range(3)])

    rays_per_cell = (cell / (az[1] - az[0])) * (cell / (el[1] - el[0]))
    fill = np.minimum(count / rays_per_cell, 1.0)
    gain_db = antenna_gain_db(a, e, cfg)

    if cfg.eirp_dbm is None:
        if cfg.snr_ref is None:
            raise ValueError("the radar model needs snr_ref or a link budget (eirp_dbm)")

        # R^-4 here, but a surface larger than the cell grows its lit patch with R^2: R^-2 (the link budget
        # below has it). surface_law 20 (radar_array --surface-law) gives that; 40 is what the avoidance
        # results were made with (docs/survey_mapping_plan.md, phase 5).
        law = getattr(cfg, "surface_law", 40.0) or 40.0
        snr = (cfg.snr_ref - law * np.log10(r / cfg.range_ref) + 10.0 * np.log10(fill))
    else:
        # Radar equation, P_r = EIRP * G_rx * lambda^2 * sigma / ((4 pi)^3 R^4), for the
        # patch of surface the cell covers: its radar cross section grows with R^2.
        patch = cell * cell * np.cos(e) * r ** 2 * fill
        sigma_db = cfg.reflectivity_db + 10.0 * np.log10(patch)
        snr = (cfg.eirp_dbm + cfg.rx_gain_dbi + 20.0 * np.log10(cfg.wavelength_mm * 1e-3)
               - 30.0 * np.log10(4.0 * np.pi) - 40.0 * np.log10(r) + sigma_db
               - cfg.noise_dbm - cfg.loss_db)

    snr = (snr + gain_db + (getattr(cfg, "surface_db", 0.0) or 0.0)      # sensitivity tests (radar_array)
           + 10.0 * cfg.specular * np.log10(np.maximum(facing, 1e-3))
           + rng.normal(0.0, cfg.snr_sigma, len(r)))

    if cfg.output == "altimeter":
        # The sensor always answers, with its SNR; the user has to drop what is below the
        # threshold. Nothing in the beam, nothing reported.
        pool = np.arange(len(r)) if cfg.pick == "strongest" else np.flatnonzero(snr > cfg.snr_threshold)
        keep = np.empty(0, dtype=np.int64)

        if len(pool):
            keep = pool[[np.argmax(snr[pool]) if cfg.pick == "strongest" else np.argmin(r[pool])]]

    else:
        # Strongest first, then at most cfg.per_range_bin cells from any one range bin.
        keep = np.flatnonzero(snr > cfg.snr_threshold)
        keep = keep[np.argsort(-snr[keep])]
        bins = np.floor(r[keep] / cfg.range_bin).astype(np.int64)
        order = np.argsort(bins, kind="stable")
        rank = np.empty(len(keep), dtype=np.int64)
        starts = np.flatnonzero(np.r_[True, np.diff(bins[order]) != 0])
        rank[order] = np.arange(len(keep)) - np.repeat(starts, np.diff(np.r_[starts, len(keep)]))
        keep = keep[rank < cfg.per_range_bin][:cfg.max_points]

    if cfg.output == "range":
        # One beam, no angle information: the nearest return, straight ahead.
        keep = keep[np.argsort(r[keep])[:1]]

    if cfg.output in ("range", "altimeter"):
        a, e = np.zeros_like(a), np.zeros_like(e)
        sigma_az = sigma_el = 0.0
    else:
        sigma_az, sigma_el = cfg.sigma_az, cfg.sigma_el

    det = measure(r[keep], a[keep], e[keep], snr[keep], velocity, cfg, rng, sigma_az, sigma_el,
                  outlier=cfg.output == "points")
    return (det, centroid[keep]) if truth else det


WIRE_STEP = 0.1             # m between the samples along a wire
WIRE_ASPECT_DB = 3.0        # dB lost at WIRE_ASPECT_DEG off the perpendicular (estimate)
WIRE_ASPECT_DEG = 30.0


def wire_detections(p0, p1, radius, sensor, cfg, rng, velocity=(0.0, 0.0, 0.0)):
    """Detections of one thin straight wire, from p0 to p1 in the radar frame (m); (N, 5) as detect().

    The ray grid cannot show a wire: it is thinner than the ray spacing (17 cm at 10 m for 1 deg),
    so a ray hits it by chance. The wire is instead sampled along its length and each sample gets
    the radar cross section of a thin cylinder seen side-on. Only a strip of length sqrt(lambda R)
    reflects in phase, and then sigma = 2 pi a L^2 / lambda = 2 pi a R: independent of the
    wavelength. The return fades away as the line of sight leaves the perpendicular to the wire
    (WIRE_ASPECT_DB at WIRE_ASPECT_DEG, an estimate: the paper reports detections mainly within
    +-30 deg). In SNR this is the cell model of detect(): snr_ref is the SNR of a flat surface
    filling one resolution cell at range_ref, whose cross section is taken as its area. Resolution
    cells and the antenna pattern are as in detect(); each cell reports its strongest sample.

    sensor: the profile's sensor section (az/el limits in deg, range_max): samples outside are dropped.
    """
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    length = np.linalg.norm(p1 - p0)
    direction = (p1 - p0) / length
    t = np.arange(0.0, length, WIRE_STEP) + WIRE_STEP / 2
    pts = p0 + t[:, None] * direction

    r = np.linalg.norm(pts, axis=1)
    a = np.arctan2(pts[:, 1], pts[:, 0])
    e = np.arcsin(np.clip(pts[:, 2] / np.maximum(r, 1e-9), -1.0, 1.0))
    inside = ((r >= max(cfg.min_range, sensor["range_min"])) & (r <= sensor["range_max"])
              & (np.degrees(a) >= sensor["az_min_deg"]) & (np.degrees(a) <= sensor["az_max_deg"])
              & (np.degrees(e) >= sensor["el_min_deg"]) & (np.degrees(e) <= sensor["el_max_deg"]))

    if not inside.any():
        return np.empty((0, 5))

    pts, r, a, e = pts[inside], r[inside], a[inside], e[inside]
    unit = pts / r[:, None]
    off_perpendicular = np.degrees(np.arcsin(np.clip(np.abs(unit @ direction), 0.0, 1.0)))
    aspect_db = -WIRE_ASPECT_DB * (off_perpendicular / WIRE_ASPECT_DEG) ** 2

    sigma = 2.0 * np.pi * radius * r
    cell = np.radians(cfg.angle_cell)

    if cfg.eirp_dbm is None:
        area_ref = (cell * cfg.range_ref) ** 2
        snr = cfg.snr_ref + 10.0 * np.log10(sigma / area_ref) - 40.0 * np.log10(r / cfg.range_ref)
    else:
        snr = (cfg.eirp_dbm + cfg.rx_gain_dbi + 20.0 * np.log10(cfg.wavelength_mm * 1e-3)
               - 30.0 * np.log10(4.0 * np.pi) - 40.0 * np.log10(r) + 10.0 * np.log10(sigma)
               - cfg.noise_dbm - cfg.loss_db)

    snr = (snr + antenna_gain_db(a, e, cfg) + aspect_db + rng.normal(0.0, cfg.snr_sigma, len(r))
           + (getattr(cfg, "thin_db", 0.0) or 0.0))                         # sensitivity tests (radar_array)

    key = np.stack((np.floor(r / cfg.range_bin), np.floor((a - np.radians(sensor["az_min_deg"])) / cell),
                    np.floor((e - np.radians(sensor["el_min_deg"])) / cell)), axis=1).astype(np.int64)
    order = np.argsort(-snr)
    _, first = np.unique(key[order], axis=0, return_index=True)
    keep = order[first]
    keep = keep[snr[keep] > cfg.snr_threshold]

    if not len(keep):
        return np.empty((0, 5))

    return measure(r[keep], a[keep], e[keep], snr[keep], velocity, cfg, rng, cfg.sigma_az, cfg.sigma_el)


def unit_vectors(a, e):
    return np.stack((np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)), axis=1)


def measure(r, a, e, snr, velocity, cfg, rng, sigma_az, sigma_el, outlier=True):
    """What the radar reports for targets at range r, azimuth a, elevation e (rad): (N, 5) x, y, z, radial
    velocity, SNR. Range and angles carry their noise; the radial velocity is that of the true direction
    (the Doppler shift does not depend on the angle estimate) plus its own noise, so a detection placed at a
    wrong angle has a radial velocity that does not fit where it is placed (radar_array's Doppler gate).
    """
    n = len(r)
    # Positive radial velocity = target receding, so closing on a static world is negative.
    v_radial = -unit_vectors(a, e) @ np.asarray(velocity, dtype=float)
    v_radial = v_radial + rng.normal(0.0, getattr(cfg, "sigma_velocity", 0.0) or 0.0, n)
    r = r + rng.normal(0.0, cfg.sigma_range, n)
    a = a + rng.normal(0.0, np.radians(sigma_az), n)
    e = e + rng.normal(0.0, np.radians(sigma_el), n)
    share = getattr(cfg, "angle_outlier", 0.0) or 0.0

    if outlier and share > 0.0:
        wrong = rng.random(n) < share
        cell = np.radians(cfg.angle_cell)
        a[wrong] += rng.uniform(-2.0, 2.0, wrong.sum()) * cell
        e[wrong] += rng.uniform(-2.0, 2.0, wrong.sum()) * cell

    return np.column_stack((r[:, None] * unit_vectors(a, e), v_radial, snr))


def fov_segments(range_m, az_min, az_max, el_min, el_max, steps=12):
    """Line segments outlining the radar's field of view at range_m, in the radar frame.

    Returns an (M, 2, 3) array. For a sector: rays from the sensor to the four
    corners, the curved border at range_m and its centre cross. For full 360 deg
    azimuth: rings at the lower and upper elevation limit and at 0, joined by
    meridians every 45 deg.
    """
    def ray(a, e):
        return range_m * np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])

    def arc(a0, e0, a1, e1, n=steps):
        t = np.linspace(0.0, 1.0, n + 1)
        pts = [ray(a0 + (a1 - a0) * s, e0 + (e1 - e0) * s) for s in t]
        return [(p, q) for p, q in zip(pts[:-1], pts[1:])]

    # A 360 deg scan stops one step short of 180 deg, so test with some margin.
    if az_max - az_min >= np.radians(350):
        segs = []

        for e in (el_min, 0.0, el_max):
            segs += arc(-np.pi, e, np.pi, e, n=4 * steps)

        for a in np.radians(np.arange(0, 360, 45)):
            segs += arc(a, el_min, a, el_max)

        return np.array(segs)

    segs = [(np.zeros(3), ray(a, e)) for a in (az_min, az_max) for e in (el_min, el_max)]
    segs += arc(az_min, el_min, az_max, el_min) + arc(az_min, el_max, az_max, el_max)
    segs += arc(az_min, el_min, az_min, el_max) + arc(az_max, el_min, az_max, el_max)
    mid_az, mid_el = (az_min + az_max) / 2, (el_min + el_max) / 2
    segs += arc(az_min, mid_el, az_max, mid_el) + arc(mid_az, el_min, mid_az, el_max)
    return np.array(segs)
