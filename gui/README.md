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
- Set sonar range (1-300 m)
- Set gain (0-40 dB)
- Set sound velocity (1300-1700 m/s)
- Trigger a ping
- Real-time status feedback

## Requirements
- ROS 2 (rclpy, std_srvs, pingdsp_msg)
- Python 3 with tkinter (usually pre-installed)

No additional dependencies needed - tkinter is built into Python!
