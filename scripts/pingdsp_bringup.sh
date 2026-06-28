#!/bin/bash
# PingDSP 3DSS-DX bringup (sim or real).
#
# Opens a tmux session with three windows:
#   1. sonar : 3DSS-DX driver (3dss.launch). MODE=real connects to the live
#              sonar over TCP; MODE=sim points the driver at the local replayer
#              and runs the pcap replay in the second pane.
#   2. viz   : Foxglove bridge + live sidescan viewer node (sidescan_viewer.launch)
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
# Usage:
#   ./pingdsp_bringup.sh                          # MODE=real (live hardware)
#   MODE=sim ./pingdsp_bringup.sh                 # replay live_sensor.pcap
#   MODE=sim SIM_PCAP=/path/to/capture.pcap ./pingdsp_bringup.sh
#   SONAR_HOST=192.168.1.100 ./pingdsp_bringup.sh
#   ENABLE_SBG=false ./pingdsp_bringup.sh
#   ENABLE_BAG=true ./pingdsp_bringup.sh          # real mode: record everything
#   ENABLE_RECORDER=true ./pingdsp_bringup.sh     # save filtered cloud to PLY/XYZ/PCD
#
# Override via environment:
#   MODE            sim | real            (default: real)
#   SONAR_HOST      sonar TCP host        (real; default: 3dss.launch default)
#   SIM_PCAP        capture to replay     (sim;  default: <pkg>/network_dump/live_sensor.pcap)
#   SIM_TCP_PORT    local sonar TCP port  (sim;  default: 23848)
#   PCAP_SPEED      replay speed mult.    (sim;  default: 1.0)
#   PCAP_LOOP       true/false            (sim;  default: true)
#   PCAP_START      skip to this t_rel(s) (sim;  default: 0; sonar starts ~18.4s
#                                          into live_sensor.pcap)
#   FOXGLOVE_PORT   foxglove bridge port  (default: 8765)
#   ENABLE_SBG      true/false            (default: true; auto-false for sonar-only pcaps)
#   ENABLE_BAG      record a bag (real)   (default: false)
#   ENABLE_RECORDER true/false            (default: false; runs pointcloud_recorder,
#                                          which writes the accumulated filtered cloud
#                                          to PLY/XYZ/PCD on Ctrl+C. Output dir/topic/
#                                          frame are set in config/recorder_params.yaml,
#                                          default <pkg>/pointclouds/.)
#   ROS_SETUP       extra setup.bash to source (optional)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# scripts -> pingdspinger -> my_pkgs -> src -> <workspace>
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

SESSION="${SESSION:-pingdsp}"
MODE="${MODE:-real}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
ENABLE_BAG="${ENABLE_BAG:-false}"
ENABLE_RECORDER="${ENABLE_RECORDER:-false}"
SIM_PCAP="${SIM_PCAP:-$PKG_DIR/network_dump/live_sensor.pcap}"
SIM_TCP_PORT="${SIM_TCP_PORT:-23848}"
PCAP_SPEED="${PCAP_SPEED:-1.0}"
PCAP_LOOP="${PCAP_LOOP:-true}"
PCAP_START="${PCAP_START:-0}"
REPLAYER="$SCRIPT_DIR/replay_pingdsp_pcap.py"

if [[ "$MODE" != "sim" && "$MODE" != "real" ]]; then
    echo "MODE must be 'sim' or 'real' (got '$MODE')." >&2
    exit 1
fi

# SBG is on by default in both modes: in sim the real sbg_device ingests the
# replayed UDP and owns the pose tree, exactly like real hardware. Only
# live_sensor.pcap carries SBG, so auto-disable it for sonar-only captures
# (the user can still force ENABLE_SBG=true if they know the capture has it).
if [[ "$MODE" == "sim" && -z "${ENABLE_SBG:-}" ]]; then
    if [[ "$(basename "$SIM_PCAP")" == live_sensor* ]]; then
        ENABLE_SBG="true"
    else
        ENABLE_SBG="false"
        echo "Note: '$(basename "$SIM_PCAP")' is a sonar-only capture; SBG disabled."
    fi
fi
ENABLE_SBG="${ENABLE_SBG:-true}"

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
    SONAR_TF_ARGS="publish_tf:=false publish_odometry:=false"
fi

# The SBG stack is identical in both modes: the real sbg_device binds UDP 24333
# and parses sbgECom, whether the packets come from the live Ellipse or the
# replayer feeding it captured datagrams.
#
# REQUIRE_NAV_FIX gates the odom datum on a full GNSS solution (solution_mode==4).
# Bench/replay captures (e.g. live_sensor.pcap) run attitude-only with NO GPS fix,
# so requiring a solution would leave the TF tree disconnected and starve
# /pingdsp/odom + /pingdsp/heading. Default it false in sim so the datum locks on
# the first fix (the recorded position may be meaningless, but attitude/heading
# and the TF chain become valid); true on real hardware.
if [[ "$MODE" == "sim" ]]; then
    REQUIRE_NAV_FIX="${REQUIRE_NAV_FIX:-false}"
else
    REQUIRE_NAV_FIX="${REQUIRE_NAV_FIX:-true}"
fi
SBG_CMD="ros2 launch pingdsp_sbg sbg.launch require_nav_solution:=$REQUIRE_NAV_FIX"

# --- Window 1 (sonar) commands, per MODE ---
REPLAY_CMD=""
if [[ "$MODE" == "real" ]]; then
    # Live hardware: connect the driver to the sonar over TCP.
    SONAR_ARGS=""
    if [[ -n "${SONAR_HOST:-}" ]]; then
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
VIEWER_CMD="ros2 launch pingdsp_driver sidescan_viewer.launch"

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

# --- Window 2: viz (foxglove + sidescan viewer) ---
tmux new-window -t "$SESSION" -n viz
tmux split-window -h -t "$SESSION:viz"
pane "$SESSION:viz.0" "$FOXGLOVE_CMD"
pane "$SESSION:viz.1" "$VIEWER_CMD"

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
