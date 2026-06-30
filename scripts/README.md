# PingDSP bringup scripts

Helpers for launching the 3DSS-DX stack, either against **live hardware** or by
**replaying a network capture (pcap)**.

| File | What it does |
|------|--------------|
| [`pingdsp_bringup.sh`](pingdsp_bringup.sh) | Opens a `tmux` session (3 windows) with the sonar driver, Foxglove bridge, control GUI and SBG stack. The one command you normally run. |
| [`replay_pingdsp_pcap.py`](replay_pingdsp_pcap.py) | The replay engine. Feeds a captured sonar TCP stream (+ SBG UDP) to the drivers on one clock. `pingdsp_bringup.sh` calls this for you in `sim` mode; you can also run it standalone. |

---

## TL;DR

```bash
# from the package root: ~/colcon_ws/src/my_pkgs/pingdspinger

# Live hardware (rig on the network):
./scripts/pingdsp_bringup.sh

# Replay the asko survey capture, looped (sonar only):
MODE=sim SIM_PCAP=network_dump/asko_survey.pcap ./scripts/pingdsp_bringup.sh

# Replay the bench capture with sonar + SBG/INS, looped:
MODE=sim ./scripts/pingdsp_bringup.sh
```

Then in another terminal:

```bash
tmux attach -t pingdsp        # watch / interact
tmux kill-session -t pingdsp  # shut the whole stack down
```

> Mouse mode is on: click a pane to focus it, scroll to read history.
> Detach without killing with `Ctrl-b d`.

---

## The tmux layout

Three windows, each split into two panes (left = the stack, right = replayer /
a useful topic tail):

1. **sonar** — `3dss.launch` (the `tdss_driver`). In `sim` the right pane runs
   the pcap replayer; in `real` it tails `/sonar/status`.
2. **viz** — Foxglove bridge (left) + the **3DSS-DX control GUI** (right). The
   GUI's **Sidescan** tab renders the live waterfall in-app (no image topic, so
   bags stay lean) and exposes every visualisation knob as a slider.
3. **sbg** — `sbg.launch`: the SBG driver + odom initializer + `sbg_to_odom`.
   In `sim` the real `sbg_device` ingests the replayed UDP, exactly like live.

---

## Configuration

Every knob has a default baked into the **`USER CONFIG`** block near the top of
`pingdsp_bringup.sh` (edit it there for a permanent change). Each one is also
overridable from the environment for a one-off run, e.g.:

```bash
MODE=sim PCAP_SPEED=4.0 ENABLE_GUI=false ./scripts/pingdsp_bringup.sh
```

| Variable | Default | Applies to | Meaning |
|----------|---------|-----------|---------|
| `MODE` | `real` | both | `real` = live hardware, `sim` = replay a pcap. |
| `SESSION` | `pingdsp` | both | tmux session name (run several at once with different names). |
| `SONAR_HOST` | _empty_ → `192.168.228.50` | real | Sonar/topside TCP host. |
| `ENABLE_BAG` | `false` | real | Record `ros2 bag record -a` into `<ws>/bags`. |
| `SIM_PCAP` | `network_dump/live_sensor.pcap` | sim | Capture to replay. `live_sensor.pcap` has sonar **and** SBG; the `asko_survey*` / other captures are **sonar-only**. |
| `SIM_TCP_PORT` | `23848` | sim | Local TCP port the driver connects to (the replayer serves the sonar bytes here). |
| `PCAP_SPEED` | `1.0` | sim | Playback speed multiplier (`2.0` = 2x, etc.). |
| `PCAP_LOOP` | `true` | sim | Loop the capture forever. |
| `PCAP_START` | `0` | sim | Skip to this capture-relative time (s). Handy: sonar starts ~18.4 s into `live_sensor.pcap`. |
| `FOXGLOVE_PORT` | `8765` | both | Foxglove bridge websocket port. |
| `ENABLE_GUI` | `true` | both | Launch the 3DSS-DX control GUI (incl. live Sidescan tab). |
| `ENABLE_SBG` | `auto` | both | `true` / `false` / `auto`. `auto` = on, but **off** for sonar-only pcaps. |
| `ENABLE_RECORDER` | `false` | both | Run `pointcloud_recorder`: dumps the filtered cloud to PLY/XYZ/PCD on `Ctrl+C` (paths in `config/recorder_params.yaml`). |
| `REQUIRE_NAV_FIX` | `auto` | both | Gate the odom datum on a full GNSS fix (`solution_mode==4`). `auto` = `false` in sim (bench captures have no fix), `true` in real. |
| `ROS_SETUP` | _empty_ | both | Extra `setup.bash` to source in every pane (optional overlay). |

---

## Recipes

```bash
# --- Live hardware ---------------------------------------------------------
./scripts/pingdsp_bringup.sh                              # default real-mode
ENABLE_BAG=true ./scripts/pingdsp_bringup.sh              # + record everything to a bag
ENABLE_RECORDER=true ./scripts/pingdsp_bringup.sh         # + save filtered pointcloud on exit
SONAR_HOST=192.168.228.50 ./scripts/pingdsp_bringup.sh    # force the sonar host

# --- Replay (sim) ----------------------------------------------------------
MODE=sim ./scripts/pingdsp_bringup.sh                     # live_sensor.pcap (sonar + SBG), looped
MODE=sim SIM_PCAP=network_dump/asko_survey.pcap ./scripts/pingdsp_bringup.sh   # asko survey, looped
MODE=sim PCAP_LOOP=false ./scripts/pingdsp_bringup.sh     # play once, then stop
MODE=sim PCAP_SPEED=4.0 ./scripts/pingdsp_bringup.sh      # 4x faster
MODE=sim PCAP_START=18 ./scripts/pingdsp_bringup.sh       # skip the dead air before the sonar starts
MODE=sim ENABLE_GUI=false ./scripts/pingdsp_bringup.sh    # headless (Foxglove only, no GUI)
MODE=sim ENABLE_SBG=false ./scripts/pingdsp_bringup.sh    # force SBG off even on live_sensor.pcap

# --- Two sessions side by side --------------------------------------------
MODE=sim SESSION=replayA SIM_PCAP=network_dump/asko_survey.pcap ./scripts/pingdsp_bringup.sh
SESSION=live ./scripts/pingdsp_bringup.sh
```

---

## Prerequisites

- A built + sourced workspace: `colcon build` then `source install/setup.bash`.
- `sim` mode needs **tshark** for pcap parsing: `sudo apt install tshark`.
- `foxglove_bridge` installed (`ros-${ROS_DISTRO}-foxglove-bridge`).

---

## Recording your own pcap

On the machine wired to the topside box (sonar head at `192.168.228.1`, Windows
control PC at `192.168.228.50`):

```bash
sudo tcpdump -i <iface> -w mysurvey.pcap        # capture the whole stream
# (replace <iface> with the wired NIC, e.g. enx4865ee175de2)
```

Drop the result in `network_dump/` and replay it with
`MODE=sim SIM_PCAP=network_dump/mysurvey.pcap ./scripts/pingdsp_bringup.sh`.

---

## Standalone replayer

`pingdsp_bringup.sh` drives this for you, but you can run it directly (e.g. to
feed a driver you launched yourself):

```bash
python3 scripts/replay_pingdsp_pcap.py <pcap> [options]
```

| Option | Default | Meaning |
|--------|---------|---------|
| `--speed` | `1.0` | Playback speed multiplier. |
| `--loop` | off | Replay on repeat. |
| `--start-offset` | `0.0` | Skip events before this capture-relative time (s). |
| `--tcp-listen-host` | `127.0.0.1` | Address for the local sonar TCP server. |
| `--tcp-listen-port` | `23848` | Port the driver connects to for sonar bytes. |
| `--no-sonar` | _(sonar on)_ | Skip the sonar TCP stream. |
| `--sonar-stream` | auto | Force a `tcp.stream` index. |
| `--sonar-src` | auto | Force the sonar source IP. |
| `--connect-timeout` | `60.0` | Seconds to wait for the driver before starting the clock. |
| `--no-sbg` | _(sbg on)_ | Skip the SBG UDP stream. |
| `--sbg-udp-port` | `24333` | SBG UDP port to extract / send to. |
| `--sbg-target` | `127.0.0.1` | Host to send SBG datagrams to. |

Example:

```bash
python3 scripts/replay_pingdsp_pcap.py network_dump/asko_survey.pcap \
    --loop --tcp-listen-port 23848 --no-sbg
```
