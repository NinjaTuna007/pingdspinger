# Design: sonar-frame / ego-track ghost filter (PingDSP driver)

**Tracked issue:** [NinjaTuna007/pingdspinger#1](https://github.com/NinjaTuna007/pingdspinger/issues/1) — backlog; implement when you have time.  
**Status:** design only (not implemented)
  
**Problem:** Hull / multipath bathymetry points that **track the vehicle** in odom/map. Offline `scripts/pointcloud/clean.py` only cleans **bundled ENU `.xyz`**, not live `sensor_msgs/PointCloud2` in **`sonar`**.

## Is this realistic?

**Yes — as a bounded upgrade, not as “perfect seabed.”**

| Goal | Realistic? | Notes |
|------|------------|--------|
| Drop body-fixed shallow outliers on each ping | **Yes** | Cheap; large win on this corpus (port shallow ridge) |
| Drop points that stay near the vehicle in odom over a few seconds | **Yes** | Sliding window + vehicle XY radius; classic “moves with me → reject” |
| Reproduce full-survey `clean.py` boat passes online | **Over-reach** | Needs dense global grid, two-pass surface/seabed percentiles; keep offline |
| Never hurt real shallow features (rocks, wrecks) | **Hard** | Require **ego-consistency** (follows platform), not “shallow alone” |
| Fix intensity waterfalls | **Out of scope** | Separate SSS QC; optional cross-hint only |

So: **try it** in `pointcloud_filter` (or a sibling node) with conservative defaults and a debug “rejected” cloud. Don’t expect to retire offline ENU cleaning for map products.

## Where to put it

Preferred: extend [`pingdsp_driver/pointcloud_filter.py`](../pingdsp_driver/pingdsp_driver/pointcloud_filter.py) + [`config/filter_params.yaml`](../pingdsp_driver/config/filter_params.yaml).

Keep publishing:

- `sonar/bathymetry` — raw  
- `sonar/bathymetry_filtered` — existing filters **+** new ego/ghost stages  
- optional `sonar/bathymetry_rejected` — ghosts only (Foxglove QC)

Do **not** change TCP parsing in `tdss_driver` for v1; filter stays downstream.

## Detection ideas (sonar + short history)

Work in **sonar** for instantaneous cues; use **TF `odom`←`sonar`** for the “tracks with me” test.

### Stage A — body-frame water-column / ridge (per ping)

After existing range/intensity/altitude/jump filters:

1. Estimate **altitude / seabed ref** from the ping: median/percentile of remaining `z`, or FBR from SSS if already computed elsewhere.
2. Reject points with `z > z_ref + margin` (e.g. 2–4 m) **and** (optional) port-only / `|y| > y_min` if hull ghosts are one-sided.
3. Optional: reject points inside a **hull box** in body frame (`y ∈ [y0,y1]`, `z` shallow) from a calibrated URDF/offset.

This catches **persistent sonar-frame ridges** without needing motion.

### Stage B — ego-track occupancy (sliding window) ← “moves with you”

1. Maintain a ring buffer of last **T seconds** (or **N pings**) of filtered points transformed to **`odom`** (or `pingdsp/odom`).
2. Maintain vehicle positions over the same window.
3. For each new point in odom: if it lies within **R_xy** of **many** recent vehicle positions (or of the trajectory polyline) **and** is **H** meters above the local along-track seabed estimate, mark as ghost.
4. Simpler proxy: voxelize odom XY near the vehicle; cells that stay occupied while the boat advances (point cloud “stuck” to base_link) → reject.

Parameters (suggested defaults to tune on Örebro):

```yaml
enable_ego_track_filter: true
ego_window_s: 8.0
ego_xy_radius_m: 2.0          # near the platform track
ego_min_hits: 5               # must recur near track
ego_min_height_above_seabed_m: 2.0
ego_port_only: false          # set true if ghosts are port-locked
```

### Stage C — optional SSS hint (later)

If port waterfall shows a stable bright near-range artifact, raise prior to reject matching bathymetry `(y,z)`. Keep off by default — couples image path to bathy path.

## What not to do in the driver

- Don’t require a full mission cloud or ENU grid (that’s `clean.py`).
- Don’t delete everything shallow — wrecks exist.
- Don’t use `transmit_power` or proprietary PingDSP-only fields as the sole cue.
- Don’t block the TCP thread; keep filter async/lightweight (numpy on each cloud is fine at ~10–20 Hz if window is bounded).

## Validation

1. Foxglove: raw vs filtered vs rejected while driving a straight line — ghosts should light up on **rejected** and vanish from **filtered**.  
2. Re-extract one bag for s2b; persistent port shallow bins in `z_m` should drop sharply.  
3. Compare offline ENU `clean.py` boat removals vs online rejects on the same stretch (overlap, not identity).  
4. Regression: harbor wall / real shallow target should mostly survive Stage B (fails ego-track test).

## Implementation sketch

```text
pointcloud_callback:
  points = read xyz+i
  points = range → intensity → altitude → consecutive   # existing
  points, ghosts = stage_A_body_watercolumn(points)
  points, ghosts2 = stage_B_ego_track(points, tf_buffer)  # new
  publish filtered; publish rejected = concat(ghosts*)
  update ring buffer in odom
```

## Relation to s2b-v2

Until this ships, approaches **mask GT in extract/train/eval** (see sidescan_to_bathy `journal/GT-GHOST-POINTS.md` + fleet `STEER.md`). After driver deploy, re-bag or re-filter and regenerate caches.
