#!/bin/bash
# PingDSP 3DSS-DX bringup (sim or real).
#
# Opens a tmux session with three windows:
#   1. sonar : 3DSS-DX driver (3dss.launch). MODE=real connects to the live
#              sonar over TCP; MODE=sim points the driver at the local replayer
#              and runs the pcap replay in the second pane.
#   2. viz   : Foxglove bridge + 3DSS-DX control GUI. The GUI's "Sidescan" tab
#              renders the live waterfall in-app and exposes every visualisation knob as a slider.
#   3. sbg   : SBG driver + odom initializer + sbg_to_odom (sbg.launch). Same in
#              both modes -- in sim the driver simply ingests the replayed UDP.
#
# Each window splits into two panes: the left pane runs the stack, the right
# pane runs the replayer (sonar window, sim) or tails a useful topic.
#
# MODE explains the data source:
#   real  the rig is on the network: sonar over TCP, SBG Ellipse over UDP 24333.
#   sim   replay a network capture (pcap). A single replayer (one clock) feeds
#         BOTH the sonar bytes (over a local TCP server tdss_driver connects to)
#         and the SBG sbgECom datagrams (UDP -> sbg_device), so sonar + INS stay
#         time-aligned. Only network_dump/live_sensor.pcap carries both streams;
#         the asko_survey*/pingDSP_traffic captures are sonar-only (ENABLE_SBG
#         is forced false for those).
#
# Configuration lives in the "USER CONFIG" block below: just edit the values
# in this file. Every knob is also overridable from the environment for one-off
# runs, e.g.  MODE=sim ENABLE_RECORDER=true ./pingdsp_bringup.sh
#
# Quick examples:
#   ./pingdsp_bringup.sh                          # MODE=real (live hardware)
#   MODE=sim ./pingdsp_bringup.sh                 # replay live_sensor.pcap
#   MODE=sim SIM_PCAP=/path/to/capture.pcap ./pingdsp_bringup.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# scripts -> pingdspinger -> my_pkgs -> src -> <workspace>
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

# ============================================================================
# USER CONFIG  -- edit these directly. Each is written as NAME="${NAME:-value}"
# so an environment variable of the same name still overrides the value here.
# ============================================================================

# --- Mode & session --------------------------------------------------------
MODE="${MODE:-real}"              # real = live hardware | sim = replay a pcap
SESSION="${SESSION:-pingdsp}"     # tmux session name

# --- Live hardware (MODE=real) ---------------------------------------------
SONAR_HOST="${SONAR_HOST:-}"      # sonar TCP host; empty = 3dss.launch default (192.168.228.50)
ENABLE_BAG="${ENABLE_BAG:-false}" # record a full `ros2 bag record -a` into <ws>/bags

# --- Replay (MODE=sim) -----------------------------------------------------
SIM_PCAP="${SIM_PCAP:-$PKG_DIR/network_dump/live_sensor.pcap}"  # capture to replay
SIM_TCP_PORT="${SIM_TCP_PORT:-23848}"  # local sonar TCP port the driver connects to
PCAP_SPEED="${PCAP_SPEED:-1.0}"        # replay speed multiplier
PCAP_LOOP="${PCAP_LOOP:-true}"         # loop the capture forever
PCAP_START="${PCAP_START:-0}"          # skip to this t_rel (s); sonar starts ~18.4s into live_sensor.pcap

# --- Common ----------------------------------------------------------------
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"       # foxglove bridge websocket port
ENABLE_GUI="${ENABLE_GUI:-true}"             # launch the 3DSS-DX control GUI (incl. live Sidescan tab)
ENABLE_SBG="${ENABLE_SBG:-auto}"             # true | false | auto (auto: on, but off for sonar-only pcaps)
ENABLE_RECORDER="${ENABLE_RECORDER:-false}"  # run pointcloud_recorder: filtered cloud -> PLY/XYZ/PCD on
                                             #   Ctrl+C (output dir/topic/frame in config/recorder_params.yaml)
REQUIRE_NAV_FIX="${REQUIRE_NAV_FIX:-auto}"   # true | false | auto (auto: false in sim, true in real)
ROS_SETUP="${ROS_SETUP:-}"                   # extra setup.bash to source (optional)

# ============================================================================
# End of user config.
# ============================================================================

REPLAYER="$SCRIPT_DIR/replay_pingdsp_pcap.py"

if [[ "$MODE" != "sim" && "$MODE" != "real" ]]; then
    echo "MODE must be 'sim' or 'real' (got '$MODE')." >&2
    exit 1
fi

# SBG is on by default in both modes: in sim the real sbg_device ingests the
# replayed UDP and owns the pose tree, exactly like real hardware. Only
# live_sensor.pcap carries SBG, so 'auto' disables it for sonar-only captures
# (set ENABLE_SBG=true/false above to force it either way).
if [[ "$ENABLE_SBG" == "auto" ]]; then
    if [[ "$MODE" == "sim" && "$(basename "$SIM_PCAP")" != live_sensor* ]]; then
        ENABLE_SBG="false"
        echo "Note: '$(basename "$SIM_PCAP")' is a sonar-only capture; SBG disabled."
    else
        ENABLE_SBG="true"
    fi
fi

if [[ "$MODE" == "sim" && ! -e "$SIM_PCAP" ]]; then
    echo "MODE=sim but capture not found: $SIM_PCAP" >&2
    echo "Set SIM_PCAP=/abs/path/to/capture.pcap (live_sensor.pcap has sonar+SBG)." >&2
    exit 1
fi

if [[ "$MODE" == "sim" ]] && ! command -v tshark >/dev/null 2>&1; then
    echo "MODE=sim needs tshark for pcap replay. Install: sudo apt install tshark" >&2
    exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "A tmux session named '$SESSION' already exists."
    echo "Attach with:  tmux attach -t $SESSION"
    echo "Or kill it:   tmux kill-session -t $SESSION"
    exit 1
fi

# Command prefix sourced into every pane so each shell has the overlay.
SOURCE_CMD="source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash"
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
    SOURCE_CMD="$SOURCE_CMD && source '$WS_DIR/install/setup.bash'"
fi
if [[ -n "${ROS_SETUP:-}" ]]; then
    SOURCE_CMD="$SOURCE_CMD && source '$ROS_SETUP'"
fi

# When the SBG stack runs it owns the map/odom/base_link TF tree and the
# vehicle odometry, so the sonar driver must not duplicate them. The sonar
# frame then hangs off pingdsp/base_link via the static TF in sbg.launch.
SONAR_TF_ARGS=""
if [[ "$ENABLE_SBG" == "true" ]]; then
    # SBG also owns the geo fix (/pingdsp/fix), so silence the sonar driver's
    # own NavSatFix to avoid two competing fix sources.
    SONAR_TF_ARGS="publish_tf:=false publish_odometry:=false publish_navsatfix:=false"
fi

# The SBG stack is identical in both modes: the real sbg_device binds UDP 24333
# and parses sbgECom, whether the packets come from the live Ellipse or the
# replayer feeding it captured datagrams.
#
# REQUIRE_NAV_FIX gates the odom datum on a full GNSS solution (solution_mode==4).
# Bench/replay captures (e.g. live_sensor.pcap) run attitude-only with NO GPS fix,
# so requiring a solution would leave the TF tree disconnected and starve
# /pingdsp/odom + /pingdsp/heading. 'auto' = false in sim so the datum locks on
# the first fix (the recorded position may be meaningless, but attitude/heading
# and the TF chain become valid); true on real hardware.
if [[ "$REQUIRE_NAV_FIX" == "auto" ]]; then
    if [[ "$MODE" == "sim" ]]; then
        REQUIRE_NAV_FIX="false"
    else
        REQUIRE_NAV_FIX="true"
    fi
fi
SBG_CMD="ros2 launch pingdsp_sbg sbg.launch require_nav_solution:=$REQUIRE_NAV_FIX"

# --- Window 1 (sonar) commands, per MODE ---
REPLAY_CMD=""
if [[ "$MODE" == "real" ]]; then
    # Live hardware: connect the driver to the sonar over TCP.
    SONAR_ARGS=""
    if [[ -n "$SONAR_HOST" ]]; then
        SONAR_ARGS="sonar_host:=$SONAR_HOST"
    fi
    SONAR_CMD="ros2 launch pingdsp_driver 3dss.launch enable_control:=true record_bag:=$ENABLE_BAG enable_recorder:=$ENABLE_RECORDER $SONAR_TF_ARGS $SONAR_ARGS"
    SONAR_STATUS_CMD="sleep 3; ros2 topic echo /sonar/status std_msgs/msg/String --field data"
else
    # Sim: run the REAL driver against a local TCP server fed from the pcap.
    # No control channel exists in a replay, so the control GUI stays off.
    # tdss_driver only retries for ~2s, so it sleeps briefly to let the
    # replayer's TCP server come up first.
    SONAR_CMD="sleep 4; ros2 launch pingdsp_driver 3dss.launch enable_control:=false sonar_host:=127.0.0.1 sonar_port:=$SIM_TCP_PORT enable_recorder:=$ENABLE_RECORDER $SONAR_TF_ARGS"
    # Second pane: the unified replayer (sonar TCP + SBG UDP on one clock). It
    # binds the TCP server immediately, then waits for the driver to connect
    # before starting the shared clock (so sonar + SBG stay aligned).
    REPLAY_ARGS="--speed $PCAP_SPEED --tcp-listen-port $SIM_TCP_PORT --start-offset $PCAP_START"
    [[ "$PCAP_LOOP" == "true" ]] && REPLAY_ARGS="$REPLAY_ARGS --loop"
    [[ "$ENABLE_SBG" != "true" ]] && REPLAY_ARGS="$REPLAY_ARGS --no-sbg"
    REPLAY_CMD="python3 '$REPLAYER' '$SIM_PCAP' $REPLAY_ARGS"
fi

FOXGLOVE_CMD="ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=$FOXGLOVE_PORT"
# Sidescan is viewed live in the control GUI's "Sidescan" tab, not as an image
# topic, so nothing here publishes sensor_msgs/Image (bags stay lean).
GUI_CMD="python3 '$PKG_DIR/gui/sonar_control_gui.py'"

SBG_ODOM_CMD="sleep 3; ros2 topic echo /pingdsp/heading std_msgs/msg/Float32 --field data"

# Helper: run a command in a pane after sourcing the overlay.
pane() {
    local target="$1"
    local cmd="$2"
    tmux send-keys -t "$target" "$SOURCE_CMD && $cmd" C-m
}

if [[ "$MODE" == "sim" ]]; then
    echo "MODE=sim: replaying '$SIM_PCAP' (speed=$PCAP_SPEED, loop=$PCAP_LOOP, sbg=$ENABLE_SBG)"
else
    echo "MODE=real: connecting to live hardware${SONAR_HOST:+ (sonar $SONAR_HOST)}"
fi

tmux -2 new-session -d -s "$SESSION" -x 220 -y 50

# --- Window 1: sonar (driver; sim adds the pcap replayer in pane 2) ---
tmux rename-window -t "$SESSION:0" sonar
tmux split-window -h -t "$SESSION:sonar"
pane "$SESSION:sonar.0" "$SONAR_CMD"
if [[ "$MODE" == "sim" ]]; then
    pane "$SESSION:sonar.1" "$REPLAY_CMD"
else
    pane "$SESSION:sonar.1" "$SONAR_STATUS_CMD"
fi

# --- Window 2: viz (foxglove + control GUI with live Sidescan tab) ---
tmux new-window -t "$SESSION" -n viz
tmux split-window -h -t "$SESSION:viz"
pane "$SESSION:viz.0" "$FOXGLOVE_CMD"
if [[ "$ENABLE_GUI" == "true" ]]; then
    pane "$SESSION:viz.1" "$GUI_CMD"
else
    pane "$SESSION:viz.1" "echo 'Control GUI disabled (ENABLE_GUI=false)'"
fi

# --- Window 3: sbg (UDP driver + odom; in sim it ingests the replayed UDP) ---
tmux new-window -t "$SESSION" -n sbg
tmux split-window -h -t "$SESSION:sbg"
if [[ "$ENABLE_SBG" == "true" ]]; then
    pane "$SESSION:sbg.0" "$SBG_CMD"
    pane "$SESSION:sbg.1" "$SBG_ODOM_CMD"
else
    pane "$SESSION:sbg.0" "echo 'SBG disabled (ENABLE_SBG=false)'"
    pane "$SESSION:sbg.1" "echo 'SBG disabled'"
fi

tmux set-option -t "$SESSION" mouse on
tmux select-window -t "$SESSION:sonar"
tmux -2 attach-session -t "$SESSION"
