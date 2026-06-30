# pingdspinger: ROS 2 stack for the PingDSP 3DSS-DX sidescan sonar

ROS 2 packages for operating a PingDSP 3DSS-DX 3D sidescan sonar together with
the SBG Ellipse INS that rides on the same rig. It covers the binary TCP data
stream, the ASCII control interface, navigation/odometry from the INS, live
sidescan visualisation, PCAP replay for offline testing, and a one-shot tmux
field bringup.

## Packages

| Package | What it does |
| --- | --- |
| `pingdsp_driver` | 3DSS-DX TCP driver (`tdss_driver`), ASCII control services (`sonar_control_node`), point-cloud filter/recorder, and an optional headless `sidescan_viewer_node` (live sidescan is normally viewed in the control GUI instead). |
| `pingdsp_msg` | `Ping3DSS` message + the sonar-control service definitions. |
| `pingdsp_sbg` | SBG Ellipse (NED) → ROS (ENU) odometry + TF: `sbg_to_odom_initializer` and `sbg_to_odom`, using `transforms3d` and `utm`. |

```text
pingdspinger/
├── pingdsp_driver/      # TCP sonar driver, control services, sidescan viewer
├── pingdsp_msg/         # Ping3DSS msg + sonar control srv
├── pingdsp_sbg/         # SBG INS -> odom/TF
├── scripts/             # pingdsp_bringup.sh (tmux field bringup)
├── docs/                # our own docs (architecture, sonar control, SBG, viewer)
├── vendor/              # vendor-supplied material (docs, installers) - tracked
└── local/              # KTH-specific / irrelevant material - gitignored
```

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the full node/topic/TF map.

## Build

```bash
cd ~/colcon_ws
colcon build --packages-up-to pingdsp_driver pingdsp_sbg
source install/setup.bash
```

Python deps (declared in each `package.xml`): `numpy`, `opencv`, `utm`,
`transforms3d`. The SBG packages also need the `sbg_driver` (sbg_ros2_driver)
on your overlay.

## One-shot bringup (sim or real)

```bash
./scripts/pingdsp_bringup.sh            # MODE=real: live hardware (default)
MODE=sim ./scripts/pingdsp_bringup.sh   # replay a network capture (pcap) instead
```

> Full launch guide — every mode, knob and recipe — is in
> [`scripts/README.md`](scripts/README.md).

Creates a tmux session `pingdsp` with three windows:

1. **sonar** – `3dss.launch` (the real driver). Real: connects to the sonar over
   TCP + ASCII control services. Sim: connects to the local replayer (control
   off), and the second pane runs the pcap replayer.
2. **viz** – Foxglove bridge and the 3DSS-DX control GUI. The GUI's **Sidescan**
   tab renders the live waterfall in-app (sliders for every knob) — *no ROS
   image topic*, so bags are not bloated (`ENABLE_GUI=false` to skip it).
3. **sbg** – `sbg.launch` (real `sbg_device` UDP driver + odom stack). Identical
   in both modes; in sim the driver simply ingests the replayed UDP datagrams.

`sim` replays a **network capture (pcap)**, not a ROS bag. A single replayer
([`scripts/replay_pingdsp_pcap.py`](scripts/replay_pingdsp_pcap.py)) feeds *both*
the sonar bytes (over a local TCP server `tdss_driver` connects to) and the SBG
sbgECom datagrams (UDP → `sbg_device`) from one capture-relative clock, so the
sonar pings and INS solution stay time-aligned. Only
`network_dump/live_sensor.pcap` carries both streams; the `asko_survey*` /
`pingDSP_traffic` captures are sonar-only, so sim auto-disables SBG for those.

Override with environment variables, e.g.:

```bash
# Real hardware, record everything to a bag
SONAR_HOST=192.168.228.50 ENABLE_BAG=true ./scripts/pingdsp_bringup.sh

# Sim: replay a specific capture at 2x, looping (default)
MODE=sim SIM_PCAP=/data/mission.pcap PCAP_SPEED=2.0 ./scripts/pingdsp_bringup.sh

# Sim: sonar-only capture (SBG auto-disabled), skip ahead to t=20s
MODE=sim SIM_PCAP=network_dump/asko_survey.pcap PCAP_START=20 ./scripts/pingdsp_bringup.sh
```

In `live_sensor.pcap` the sonar TCP stream only begins ~18.4s in (the SBG INS
streams from the start), so expect a short delay before pings appear — or set
`PCAP_START` to skip ahead. For low-level validation of just the SBG UDP driver
(raw pcap → `sbg_device`), use `ros2 launch pingdsp_sbg test_sbg.launch
pcap_file:=...` (see [`docs/SBG_INTEGRATION.md`](docs/SBG_INTEGRATION.md)).

## Running pieces individually

```bash
# Sonar driver + ASCII control services
ros2 launch pingdsp_driver 3dss.launch                 # set sonar_host in 3dss_params.yaml

# Control GUI (incl. live Sidescan tab — renders in-app, no image topic)
python3 gui/sonar_control_gui.py

# SBG INS -> /pingdsp/odom + TF (utm -> pingdsp/odom -> pingdsp/base_link)
ros2 launch pingdsp_sbg sbg.launch
```

### Sidescan viewing (runtime-tunable, no topic)

The driver no longer publishes a rendered sonar image (it bloated bags). The
waterfall is rendered **live in the control GUI's Sidescan tab** straight from
the raw `Ping3DSS` samples, with a slider for every knob (pings, width, log
min/max, gamma, nadir, flatten, CLAHE, despeckle, colormap) plus Reset/Save/
Pause. Nothing is published, so the raw samples in the bag are the only sonar
data recorded.

An optional headless `sidescan_viewer_node` can still publish the waterfall as
`sonar/sidescan_image` for Foxglove/rviz if you want it (it is *not* started by
the bringup, and a `ros2 bag record -a` would capture that image):

```bash
ros2 launch pingdsp_driver sidescan_viewer.launch
ros2 param set /sidescan_viewer_node num_pings 500
```

Details: [`docs/SIDESCAN_VIEWER.md`](docs/SIDESCAN_VIEWER.md).

### Sonar control

```bash
ros2 service call /sonar/set_range pingdsp_msg/srv/SetSonarRange "{range: 75.0}"
```

Full command/service reference: [`docs/SONAR_CONTROL.md`](docs/SONAR_CONTROL.md).

### SBG / odometry

The SBG Ellipse streams sbgECom over **UDP** here (port 24333). See
[`docs/SBG_INTEGRATION.md`](docs/SBG_INTEGRATION.md) for the frame conventions,
the NED→ENU math, and how to verify the path against a recorded capture.

## Recording captures (pcap / raw bin)

The rig is a 3DSS-iDX-FULL on a `192.168.228.0/24` LAN behind the SIU's built-in
Ethernet switch (head `.1`, Windows 3DSS-DX Control `.50`, this host `.69`). The
full vendor-validated IP/port map is in
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md#network-topology-ips--ports). The
captures under `network_dump/` were made two ways:

```bash
# Raw sonar TCP stream only (the *.bin dumps): pull the 3DSS-DX data port.
nc 192.168.228.50 23848 > out.bin

# Full network capture (the *.pcap files): sniff a whole interface.
sudo tcpdump -i enx4865ee175de2 -w asko_survey.pcap
```

> **Capture the right interface or you lose the SBG.** The SBG Ellipse
> broadcasts sbgECom to the link-local address `255.255.255.255:24333` from
> `192.168.228.1` (the sonar head, on the SIU's switched segment). That broadcast is
> **not forwarded** to the topside segment, so a `tcpdump` on the topside
> adapter (`enx…`, which only sees the host ↔ 3DSS Control `.50` sonar traffic)
> records the sonar but **not** the SBG. This is why `asko_survey*.pcap` /
> `pingDSP_traffic.pcap` contain the sonar (with a real GPS fix embedded as NMEA
> in the ping stream) but no UDP 24333, while `live_sensor.pcap` — captured where
> the broadcast was visible — has the SBG UDP but no GPS fix (bench run).
> To capture both in future, sniff the segment the SBG broadcasts on (or point
> the SBG's output at the capture host), and confirm with
> `tshark -r cap.pcap -Y "udp.port==24333" -c1`.

## Offline replay (no hardware)

```bash
# Unified: sonar TCP + SBG UDP from one capture, on a single clock.
# (This is what MODE=sim drives; run it standalone for ad-hoc testing.)
python3 scripts/replay_pingdsp_pcap.py network_dump/live_sensor.pcap --speed 10
#   -> serves sonar bytes on 127.0.0.1:23848 and sends SBG UDP to 127.0.0.1:24333.
#   Point tdss_driver at 127.0.0.1:23848 and run sbg.launch to consume both.

# Sonar only (sonar-only captures): omit the SBG half.
python3 scripts/replay_pingdsp_pcap.py network_dump/asko_survey.pcap --no-sbg --speed 10

# SBG UDP only, end-to-end with the odom stack via the test launch
ros2 launch pingdsp_sbg test_sbg.launch pcap_file:=/abs/path/to/capture.pcap
```

## Testing

Both packages ship pure unit tests (no hardware) plus full-stack integration
tests that emulate the sonar (in-process TCP server streaming synthetic DX
frames) and the SBG (synthetic NED messages):

```bash
# Pure logic only, no build required:
cd pingdsp_driver && PYTHONPATH=. python3 -m pytest test/test_nav_parsers.py \
    test/test_dx_structures.py test/test_sidescan_image.py
cd pingdsp_sbg && PYTHONPATH=. python3 -m pytest test/test_sbg_transforms.py

# Everything (integration tests auto-skip unless built/sourced):
colcon test --packages-select pingdsp_driver pingdsp_sbg
colcon test-result --verbose
```

## Troubleshooting

```bash
# Sonar reachable? (3DSS-DX Control data stream, TCP 23848; the head is .1,
# the Windows PC running 3DSS-DX Control is .50 -- see docs/ARCHITECTURE.md)
nc -zv <sonar_host> 23848
ros2 param get /tdss_driver sonar_host

# Data flowing?
ros2 topic hz /sonar/ping
ros2 topic echo /sonar/status std_msgs/msg/String --field data

# SBG present in a capture? (sbgECom UDP, port 24333)
tshark -r network_dump/capture.pcap -Y "udp.port == 24333" | head
```

## License

Apache-2.0

## Author

Shekhar Devm Upadhyay, based on the PingDSP 3DSS-DX Structure API and the
evolo_common SBG odometry design.
