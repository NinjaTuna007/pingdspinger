# Architecture

End-to-end view of the `pingdspinger` stack: the 3DSS-DX sonar, the SBG Ellipse
INS, and the ROS 2 nodes that turn them into topics, images, odometry, and TF.

## Hardware interfaces

| Source | Transport | Consumer |
| --- | --- | --- |
| 3DSS-DX data stream | TCP 23848 | `tdss_driver` |
| 3DSS-DX control | TCP 23840 (ASCII commands) | `sonar_control_node` |
| SBG Ellipse INS | UDP 24333 (sbgECom) | `sbg_driver/sbg_device` |

See [Network topology](#network-topology-ips--ports) below for the full,
vendor-validated IP/port map.

## Network topology (IPs & ports)

Our rig is a **3DSS-iDX-FULL**: the SBG Ellipse-3 INS lives inside the sonar head
and the head itself emits the INS solution (hence SBG UDP comes from `.1`, the
iDX-Base/FULL case, not from the SIU's `.92` Navsight as on an iDX-PRO). The
physical layout, fully validated against the vendor docs (UM002 Quick Start
§4/§12, UM005 AUV Integration Guide) and our own packet captures:

```
            sonar head  192.168.228.1  (fixed)
            3DSS-DX electronics + SBG Ellipse-3 INS
                          │  (single sonar cable)
                          ▼
      ┌───────────────────────────────────────────┐
      │  SIU "Sonar Interface Unit" 192.168.228.90 │  ← the topside blue box
      │  (integrated Ethernet switch, ports E1/E2)  │     UM002 §5
      └───────┬───────────────────────────┬─────────┘
              │ E1                          │ E2
              ▼                             ▼
   Windows acquisition PC            this machine (2nd computer)
   192.168.228.50                    192.168.228.69
   runs 3DSS-DX Control              ROS 2 stack / packet capture
   (the TCP 23848 server)            UM002 §5.7 allows a 2nd PC on E1/E2
```

`.50` and `.69` are both inside the documented static-PC range
(`192.168.228.2`–`.89`, /24, gateway empty, interface metric 5; UM002 §4). The
SIU optionally also exposes `192.168.228.91` (Septentrio GNSS) and
`192.168.228.92` (SBG Navsight) for iDX-PRO builds — present in the addressing
scheme but not used by our iDX-FULL.

### Addresses (UM002 §4)

| IP | Device | Notes |
| --- | --- | --- |
| `192.168.228.1` | 3DSS-DX sonar head (+ SBG Ellipse-3 INS) | fixed, not changeable |
| `192.168.228.2`–`.89` | acquisition PC(s), static | our `.50` (Windows) and `.69` (this machine) |
| `192.168.228.90` | SIU — topside box, web UI `http://192.168.228.90` | integrated Ethernet switch |
| `192.168.228.91` | Septentrio GNSS receiver | iDX-PRO only (unused here) |
| `192.168.228.92` | SBG Navsight INS processor | iDX-PRO only (unused here) |

### Ports

| Port | Proto | Source → role | Reference |
| --- | --- | --- | --- |
| `23848` | TCP | `.50:23848` (3DSS-DX Control) → sonar data stream: 3D/2D sidescan, bathymetry, GNSS position (NMEA-0183), motion (TSS) | UM002 §12; struct API `kTcpPort` |
| `23840` | TCP | 3DSS-DX Control command/console interface (ASCII) | Control Command Interface Guide |
| `23841` | UDP | NMEA insertion into the sonar (external nav in) | UM005 AUV Integration Guide |
| `24333` | UDP | SBG INS EKF solution (sbgECom) — from `.1` on iDX-Base/FULL, from `.92` on iDX-PRO | UM002 §12 |

Two distinct nav sources exist by design (UM002 §12): the TCP `23848` stream
carries the **raw GNSS** position/motion ("position from the GNSS, not the EKF
solution from the INS"), while UDP `24333` carries the **INS EKF** solution. The
ROS stack uses the SBG EKF (`sbg_device` → odom) for pose and only mines the
TCP-embedded NMEA as a standalone fallback (`tdss_driver` with `publish_tf`).

### Why captures differ (recording caveat)

The SBG UDP `24333` stream is a link-local broadcast from the head (`.1`); you
only record it if the capturing interface sits on the **same L2 segment as the
SIU switch**. This explains our captures:

- `live_sensor.pcap` — taken with the recorder on the SIU switch, so it contains
  both the `.1`→broadcast SBG UDP `24333` *and* the `.50:23848` TCP stream.
- `asko_survey.pcap` (and other survey captures) — taken on a segment that only
  saw the relayed `.50:23848` TCP data (the head `.1` and its broadcast never
  appear, no Xilinx MAC in the capture). These still carry a valid GPS fix, but
  only as the **NMEA embedded in the TCP stream**, not the SBG EKF datagrams.

How the captures were recorded is documented in the project
[README](../README.md#recording-captures).

```
                 PingDSP 3DSS-DX                          SBG Ellipse (UDP)
              ┌────────┴────────┐                              │
       TCP data         TCP control                      sbg_device
         │                  │                                  │  /pingdsp/sbg/*
         ▼                  ▼                          ┌───────┴────────┐
   tdss_driver       sonar_control_node                │                │
     │  publishes:      services:                sbg_to_odom_     sbg_to_odom
     │  /sonar/ping     /sonar/set_range          initializer        │
     │   (Ping3DSS)     /sonar/set_gain               │ static TF     │ publishes:
     │  /sonar/bathymetry  /sonar/set_power      utm_{z}_{b}→utm   /pingdsp/odom
     │  /sonar/sidescan3d  /sonar/set_sound_velocity →pingdsp/odom   (Odometry)
     │   (PointCloud2)  /sonar/get_settings                          + dynamic TF
     │  /sonar/settings /sonar/set_trigger_mode                      pingdsp/odom→
     │  /sonar/altitude                                               pingdsp/base_link
     │  /sonar/pose  /sonar/nmea  /sonar/fix                         + /pingdsp/heading
     │  /sonar/status  /sonar/*_temperature  /diagnostics              /pingdsp/course
     ▼                                                                 /pingdsp/speed
 sidescan_viewer_node (subscribes /sonar/ping)                        /pingdsp/latlon
     │  publishes /sonar/sidescan_image (Image, on demand)
     ▼
 Foxglove / rviz2 / rosbag
```

Note: `tdss_driver` no longer publishes a rendered sonar image. Visualisation is
fully decoupled into `sidescan_viewer_node` so bags stay small (see
[`SIDESCAN_VIEWER.md`](SIDESCAN_VIEWER.md)).

### `tdss_driver` topics

| Topic | Type | Content |
|---|---|---|
| `sonar/ping` | `pingdsp_msg/Ping3DSS` | Raw port/starboard sidescan amplitudes + per-ping metadata. Bin spacing is `sound_velocity_bulk / (2 · sample_rate)`. |
| `sonar/bathymetry` | `sensor_msgs/PointCloud2` `x y z intensity quality` | The head's bottom-tracked bathymetry in the `sonar` frame. `quality` is the head's per-point sample count (vendor `reserved1`, uint32 1–200ish; higher = more 3D samples binned into the point). Published only while subscribed. |
| `sonar/sidescan3d` | `sensor_msgs/PointCloud2` `x y z intensity snr` | The full sidescan-3D point set the head computes: every range step, up to `sidescan3d_number_of_angles` angle solutions per range, water column included, with per-point SNR in dB. `sonar/bathymetry` is its bottom-tracked, binned subset. ~10× more points than bathymetry at long range (~4300/ping at 150 m, ~86 KB/ping). Published only while subscribed; `publish_sidescan3d:=false` removes the publisher. |
| `sonar/settings` | `pingdsp_msg/SonarSettings` (latched) | Every DxParameters/DxSystemInfo value in effect: range, sample rate, bin spacing, TVG gain polynomial, pulse / beamwidth / power / transmit angle per side, trigger, 2D/3D processing settings, sound velocity. Transient-local, depth 1, re-sent only when a setting changes, so a bag started mid-run still records it. |
| `sonar/altitude` | `pingdsp_msg/SonarAltitude` | The head's own nadir depth from its internal `$PDNDE` sentence (`altitude = -nadir_depth`), one per ping. |
| `sonar/water_temperature` | `sensor_msgs/Temperature` | From the AML SV probe (`$PDSVM`), ~every other ping. |
| `sonar/mcu_temperature` | `sensor_msgs/Temperature` | Head MCU temperature (`$PDHXT`). |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `3dss_dx/<sonar_id>` status with SV probe, temperatures and `$PDHXP` power-rail readings as key/values. Published only while subscribed. |
| `sonar/fix` | `sensor_msgs/NavSatFix` | GGA fix from the sonar stream. When `$GPGST` is present the `position_covariance` is `DIAGONAL_KNOWN` with east/north/up variances from the GST sigmas. |
| `sonar/pose`, `vehicle/position`, `vehicle/path` | geometry/nav msgs | Nav derived from the embedded NMEA/TSS1 (standalone mode). |
| `sonar/nmea` | `std_msgs/String` | Raw ASCII block of each ping. |
| `sonar/status`, `sonar/delivery_latency`, `sonar/rx_backlog_bytes` | String / Float32 / Int32 | Connection and stream-health telemetry. `sonar/status` also reports dropped corrupt frames. |

Every data topic is published while at least one subscriber exists; `ros2 bag record -a`
(what `record_bag:=true` runs) is such a subscriber, so a bag captures all of the above,
including the latched `sonar/settings` (verified by `test_bag_record_all_captures_every_driver_topic`).

**Frame integrity.** Before anything is published, `DxData.frame_problems()` rejects frames
whose body contains another frame's preamble (upstream byte loss splices the next frame in at
the advertised length), whose sidescan grid is not `i·c/(2fs)`, or whose point sections carry
non-finite values, ranges outside 1.5× the range setting, angles beyond ±π, or garbage quality
counts. Such frames are dropped whole (throttled warning, count in `sonar/status`). On a live
TCP link this never fires (TCP is lossless; a 35 min Örebro survey had 0 corrupt frames and 0
garbage points); it matters for pcap replay of captures with tcpdump loss, where ~1 % of frames
are spliced.

## TF tree

```
utm_{zone}_{band}            (e.g. utm_30_U)  -- static, identity
        └── utm                                -- static
              └── pingdsp/odom                 -- static, = UTM datum offset
                    └── pingdsp/base_link      -- dynamic, from sbg_to_odom
                          └── sonar            -- static mount (base_link→sonar)
```

The SBG stack owns this entire tree. The datum (`utm → pingdsp/odom`) is locked
once by `sbg_to_odom_initializer` from the first full SBG navigation fix and
re-broadcast on a timer for late joiners; `sbg.launch` adds the static
`pingdsp/base_link → sonar` mount so the sonar sensor frame hangs off the
SBG-driven vehicle frame.

To avoid two pose sources, `tdss_driver` runs with `publish_tf:=false
publish_odometry:=false` whenever the SBG stack is up (the bringup sets this), so
it only emits sensor data in the `sonar` frame. Standalone (`3dss.launch` with
its `publish_tf:=true` default) it instead derives its own `map → odom → sonar`
tree from the NMEA/TSS1 nav embedded in the sonar stream.

## Coordinate conventions

* The sonar driver projects GPS fixes to UTM with the `utm` library, locking the
  zone on the first fix; headings are converted NED→ENU and de-rotated by the
  grid (meridian) convergence (`nav_parsers`).
* The SBG runs in its native NED frame; `pingdsp_sbg.sbg_transforms` converts
  attitude, velocity and angular rate to ROS ENU/FLU via `transforms3d`
  (see [`SBG_INTEGRATION.md`](SBG_INTEGRATION.md)).

## Data flow summary

```
Sonar TCP ─► tdss_driver ─► /sonar/ping ─► sidescan_viewer_node ─► /sonar/sidescan_image
                          ├► /sonar/bathymetry ─► pointcloud_filter ─► /sonar/bathymetry_filtered
                          ├► /sonar/sidescan3d, /sonar/settings (latched), /sonar/altitude
                          └► /sonar/pose, /sonar/fix, /sonar/nmea, /sonar/status,
                             /sonar/{water,mcu}_temperature, /diagnostics

SBG UDP ─► sbg_device ─► /pingdsp/sbg/{ekf_nav, ekf_euler, imu_short, ...}
              ├► sbg_to_odom_initializer ─► static TF datum
              └► sbg_to_odom ─► /pingdsp/odom (+ dynamic TF, heading/course/speed/latlon)
                 + /pingdsp/fix (sensor_msgs/NavSatFix, for Foxglove Map)
                 (attitude from ekf_euler, gyro from imu_short; no ekf_quat needed)
```

## Deployment scenarios

```bash
# Full rig (sonar + SBG + viz) against live hardware, one tmux session
./scripts/pingdsp_bringup.sh

# Same stack, but driven by a network capture (sonar TCP + SBG UDP, one clock)
MODE=sim ./scripts/pingdsp_bringup.sh

# Sonar only, no control interface
ros2 launch pingdsp_driver 3dss.launch enable_control:=false

# Offline: replay just the SBG UDP into the odom stack
ros2 launch pingdsp_sbg test_sbg.launch pcap_file:=/abs/path/capture.pcap
```
