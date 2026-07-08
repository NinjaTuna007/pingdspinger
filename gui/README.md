# Sonar Control GUI

Simple standalone GUI for controlling the PingDSP 3DSS-DX sonar via ROS 2 services.

## Usage

1. Source your ROS 2 workspace:
   ```bash
   source install/setup.bash
   ```

2. Make sure the sonar control node is running:
   ```bash
   ros2 launch pingdsp_driver 3dss.launch enable_control:=true
   ```

3. Run the GUI:
   ```bash
   python3 gui/sonar_control_gui.py
   ```
   Or from anywhere after sourcing:
   ```bash
   cd ~/colcon_ws/src/my_pkgs/pingdspinger
   ./gui/sonar_control_gui.py
   ```

## Features
- Tabs for app/sonar control, acquisition, transmit, processing, sidescan3d,
  bathymetry, sound velocity, file, record and baud settings (all via the
  `/pingdsp/sonar/*` services from `sonar_control_node`). The GUI joins the
  `/pingdsp` namespace by default (override with `PINGDSP_NS=...`), so it uses
  relative service/topic names that resolve under that prefix.
- **Timing readout** — the top bar shows the sonar stream's delivery lag
  (`now - embedded acquisition time`, from `sonar/delivery_latency`) and the
  skew versus the SBG pose (derived here from the `sonar/ping` and `odom`
  embedded stamps — the driver stays independent of the SBG).
- **Sidescan tab** — a live sidescan waterfall rendered *inside the GUI* from the
  raw `Ping3DSS` samples (`sonar/ping`). It publishes **no image topic** (so bags
  are not bloated), and every visualisation knob is a live slider: pings, width,
  log min/max, gamma, nadir bins, flatten, CLAHE, despeckle, refresh rate and
  colormap, plus Reset / Save PNG / Pause. Rendering only runs while the tab is
  visible.
- Real-time status feedback + console log.

## Requirements
- ROS 2 (rclpy, std_srvs, pingdsp_msg) and, for the Sidescan tab, the built
  `pingdsp_driver` package (for `pingdsp_driver.sidescan_image`) on the path.
- Python 3 with tkinter (usually pre-installed).
- For the Sidescan tab only: `numpy`, `Pillow` (PIL) and `opencv-python` (cv2,
  used by the optional CLAHE/despeckle and Save PNG). If any are missing the
  control tabs still work and the Sidescan tab explains what is unavailable.
