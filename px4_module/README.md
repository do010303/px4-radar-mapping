# px4_module: PX4 code of this example, kept out of the PX4 tree

PX4 builds this directory as an external module (`EXTERNAL_MODULES_LOCATION`), so nothing
of it is inside `src/` or `boards/` and no other build of PX4 contains it. Build it with
`../scripts/build_px4.sh`: it writes `build/px4_sitl_mapping/`, next to and never over
`build/px4_sitl_default/` that the other examples run.

| Module | What |
|---|---|
| `us_d1_radar` | Ainstein US-D1 radar altimeter used as a forward distance sensor: up to three instances (one serial line and one rotation each), the manual's SNR rule, the 43 x 30 deg field of view. A new driver, not a change to PX4's `ulanding_radar`. |
