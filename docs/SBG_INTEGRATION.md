# SBG Ellipse integration

The PingDSP rig carries an SBG Ellipse INS. We run the upstream `sbg_ros2_driver`
(`sbg_device`) and convert its native NED stream into a ROS ENU odometry + TF
tree with two small nodes in `pingdsp_sbg`. The design mirrors evolo_common's
`sbg_to_odom` / `sbg_to_odom_initializer`, but the rotation math uses
`transforms3d` (not `tf_transformations`) and UTM uses the `utm` library, kept in
a ROS-free, unit-tested module (`pingdsp_sbg/sbg_transforms.py`).

## Transport: UDP

The Ellipse on this rig streams sbgECom over **UDP** (port 24333), not a serial
line. This is the vendor-documented INS interface: UM002 Quick Start §12 lists
the SBG INS solution at `192.168.228.1:24333/UDP` for an iDX-Base/FULL (or
`192.168.228.92:24333` on an iDX-PRO) — ours is an **iDX-FULL**, so the head at
`.1` is the source. See the
[network topology](ARCHITECTURE.md#network-topology-ips--ports) for the full map.
The driver config lives at `pingdsp_sbg/config/sbg_device_udp.yaml`. Verify a
recorded capture actually contains the SBG stream before expecting odometry:

```bash
tshark -r network_dump/<capture>.pcap -Y "udp.port == 24333" | head
```

If you see periodic UDP packets on 24333, the path is good. **Many of our
captures have none** — the Ellipse broadcasts to the link-local
`255.255.255.255` on the sonar's internal segment, so a topside
`tcpdump -i enx…` (between the host and 3DSS Control `.50`) records the sonar but
misses the SBG. Only `live_sensor.pcap` was captured where the broadcast was
visible; `asko_survey*` / `pingDSP_traffic` are sonar-only (their GPS fix instead
rides as NMEA inside the ping stream, parsed by `tdss_driver`). See the
"Recording captures" section of the top-level README. Replay it end-to-end:

```bash
ros2 launch pingdsp_sbg test_sbg.launch pcap_file:=/abs/path/<capture>.pcap
```

This brings up `sbg_device` (listening on `0.0.0.0:24333`), the initializer and
`sbg_to_odom`, and replays the pcap's UDP datagrams to `127.0.0.1:24333` with the
original timing (`scripts/replay_sbg_udp.py`).

To replay the **whole rig** (sonar + SBG together, time-aligned on one clock) use
`MODE=sim ./scripts/pingdsp_bringup.sh`, which drives
`scripts/replay_pingdsp_pcap.py` against `network_dump/live_sensor.pcap` — the
only capture that contains both the sonar TCP stream and the SBG UDP.

> The Ellipse broadcasts to `255.255.255.255:24333` from `192.168.228.1` (the
> sonar head on the SIU's switched segment; UM002 §12). `replay_sbg_udp.py`
> re-emits the payloads
> as unicast to `127.0.0.1:24333`, which the driver (bound to `0.0.0.0:24333`)
> still ingests. A benign `SBG_WRITE_ERROR` on startup is expected: with
> `confWithRos: false` / `in_port: 0` the driver is listen-only and cannot query
> the device, but it parses the incoming stream fine.

## What the device actually streams (important)

`confWithRos: false`, so the driver **never pushes** the `output.log_*` settings
in the YAML to the Ellipse — those lines are inert. The set of logs on the wire
is whatever is **saved on the device** (configured via SBG's tool / web GUI).
Enumerating a real PingDSP capture (`network_dump/live_sensor.pcap`) shows this
device streams a full EKF estimate, but in the Euler/short forms:

| sbgECom log | ROS topic | present? |
| --- | --- | --- |
| `EKF_NAV` (position + velocity) | `sbg/ekf_nav` | yes (~44 Hz) |
| `EKF_EULER` (roll/pitch/yaw attitude) | `sbg/ekf_euler` | yes (~44 Hz) |
| `IMU_SHORT` (gyro/accel) | `sbg/imu_short` | yes (~180 Hz) |
| `SHIP_MOTION`, `GPS1_*`, `STATUS`, `UTC_TIME` | `sbg/*` | yes |
| `EKF_QUAT` (quaternion attitude) | `sbg/ekf_quat` | **no** |
| `IMU_DATA` (legacy) | `sbg/imu_data` | **no** |

So the EKF estimate *is* produced — attitude just arrives as **Euler angles**
(`EKF_EULER`) rather than the redundant quaternion (`EKF_QUAT`), and IMU arrives
as the modern `IMU_SHORT` rather than the deprecated `IMU_DATA`. **No web-GUI
change is required**: `sbg_to_odom` consumes whichever is on the wire
(`attitude_source: auto`). If you ever want native quaternion output you can
enable `EKF_QUAT` in the SBG configuration tool and save it to the device, but it
buys nothing here.

## Nodes

### `sbg_to_odom_initializer`

Waits for the first `SbgEkfNav` with a full navigation solution
(`status.solution_mode == 4`), converts its lat/lon to UTM, and broadcasts a
**static** TF chain:

```
utm_{zone}_{band}  →  utm  →  pingdsp/odom
```

`utm_{zone}_{band} → utm` is identity; `utm → pingdsp/odom` carries the UTM
easting/northing **and altitude** of the datum. The transforms are re-sent on a
timer so late subscribers and bag splits still receive them.

### `sbg_to_odom`

* Looks up `utm → pingdsp/odom` once to recover the datum offset (easting,
  northing, and altitude).
* Per `SbgEkfNav`: UTM-forwards lat/lon, subtracts the datum → local ENU `x`/`y`.
  Vertical: with `use_altitude:=true` (default) sets
  `z = altitude − datum_altitude`; with `use_altitude:=false` freezes `z` at 0
  (2.5D). Also rotates NED ground velocity into the FLU body frame.
* Attitude (`attitude_source`): from `SbgEkfQuat` when present, otherwise from
  `SbgEkfEuler` — both converted NED→ENU through the same math (Euler is first
  turned into the identical NED attitude quaternion). `auto` prefers a live quat
  and falls back to Euler, which is what the PingDSP Ellipse needs.
* Gyro: from `SbgImuData` or `SbgImuShort` (`delta_angle`), converted FRD → FLU.
* On a timer publishes `nav_msgs/Odometry` (`pingdsp/odom → pingdsp/base_link`),
  the matching dynamic TF, plus `pingdsp/heading`, `pingdsp/course`,
  `pingdsp/speed` and `pingdsp/latlon`. Pose/twist covariance is filled from the
  SBG 1-sigma accuracy fields.
* Per `SbgEkfNav` fix it also republishes the raw geographic fix as
  `sensor_msgs/NavSatFix` on `pingdsp/fix` (lat/lon/alt + diagonal covariance
  from the position accuracy). This is what Foxglove's **Map** panel plots, and
  it streams even before the local odom datum locks (the fix is absolute).
  Toggle with `publish_navsatfix` / rename with `navsatfix_topic`.

## Frame conventions (the math)

All implemented in `pingdsp_sbg/sbg_transforms.py`:

* `R_NED_TO_ENU = [[0,1,0],[1,0,0],[0,0,-1]]` — swap North/East, flip Down.
* `R_SBG_TO_ROS = Rx(π)` — body Forward-Right-Down → Forward-Left-Up.
* Attitude: `R_enu = R_NED_TO_ENU · R_sbg · R_SBG_TO_ROS`, then `mat2quat`.
* Velocity: `v_body = R_sbg.T · v_ned`, then negate y, z (→ FLU).
* Gyro: negate y, z (FRD → FLU).
* Heading: `90 − yaw_enu_deg`, wrapped to `[0, 360)`.

`transforms3d` quaternions are `(w, x, y, z)`; ROS quaternions are `(x, y, z, w)`.
The helpers take/return ROS order and reorder internally.

Sanity checks (also asserted in `test/test_sbg_transforms.py`):

| Input (NED) | Output |
| --- | --- |
| identity attitude (level, pointing North) | ENU quat `(0,0,√½,√½)`, heading `0°` |
| yaw `+90°` (pointing East) | heading `90°` |
| ground velocity North `1 m/s`, identity attitude | body `(1, 0, 0)` forward |

## Parameters

`pingdsp_sbg/config/sbg_odom_params.yaml` configures both nodes:
`frame_prefix` (default `pingdsp`), the input topics (`sbg/ekf_nav`,
`sbg/ekf_quat`, `sbg/ekf_euler`, `sbg/imu_data`, `sbg/imu_short`),
`attitude_source` (`auto`/`quat`/`euler`), `publish_rate`, `publish_tf`,
`publish_covariance`, `use_altitude` (default `true` — local ENU `z` from EKF
altitude; set `false` to freeze `z`), and the initializer's
`require_nav_solution` / `update_rate`. Bringup passes
`use_altitude:=$USE_ALTITUDE` (default true).

Everything runs under the `pingdsp` namespace (set in `sbg.launch`), so the SBG
driver publishes `/pingdsp/sbg/*` and odometry comes out on `/pingdsp/odom`. TF
frame names are global and are not namespaced.

## Who owns the TF tree

The SBG stack is the **single authoritative source** of the vehicle pose and the
world TF tree:

```
utm_{zone}_{band} → utm → pingdsp/odom → pingdsp/base_link → sonar
```

`sbg.launch` publishes the whole chain, including a static
`pingdsp/base_link → sonar` mount transform (`publish_sonar_tf`, with
`sonar_x/y/z/roll/pitch/yaw` for the measured lever arm). To avoid two nodes
fighting over the tree, the **sonar driver does not publish TF/odometry when the
SBG stack runs**: the bringup launches `3dss.launch` with
`publish_tf:=false publish_odometry:=false`, and `tdss_driver` then only emits
sensor data (`Ping3DSS`, point clouds, NMEA) in its `sonar` frame. Run
`3dss.launch` standalone (defaults `publish_tf:=true`) if you want the driver's
own NMEA-derived `map → odom → sonar` tree without the SBG.

## Launch files

| File | Purpose |
| --- | --- |
| `sbg.launch` | Full stack: `sbg_device` + initializer + `sbg_to_odom` + static `base_link → sonar`. |
| `test_sbg.launch` | `sbg.launch` + UDP pcap replay for offline verification. |
