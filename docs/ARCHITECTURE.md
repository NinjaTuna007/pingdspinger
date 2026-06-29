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
     │   (PointCloud2)  /sonar/set_sound_velocity   →pingdsp/odom    (Odometry)
     │  /sonar/pose     /sonar/get_settings                          + dynamic TF
     │  /sonar/nmea     /sonar/set_trigger_mode                      pingdsp/odom→
     │  /sonar/status                                                 pingdsp/base_link
     │                                                               + /pingdsp/heading
     ▼                                                                 /pingdsp/course
 sidescan_viewer_node (subscribes /sonar/ping)                        /pingdsp/speed
     │  publishes /sonar/sidescan_image (Image, on demand)            /pingdsp/latlon
     ▼
 Foxglove / rviz2 / rosbag
```

Note: `tdss_driver` no longer publishes a rendered sonar image. Visualisation is
fully decoupled into `sidescan_viewer_node` so bags stay small (see
[`SIDESCAN_VIEWER.md`](SIDESCAN_VIEWER.md)).

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
                          └► /sonar/bathymetry, /sonar/pose, /sonar/nmea, /sonar/status

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
