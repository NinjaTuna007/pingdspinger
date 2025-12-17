# pingdsp_msg

Custom ROS 2 message definitions for PingDSP 3DSS-DX sonar data.

## Messages
- `Ping3DSS.msg`: Main sonar ping data
- `SonarSettings.msg`: Sonar configuration
- `SystemInfo.msg`: System information

## Usage
- Add `pingdsp_msg` as a dependency in your ROS 2 package to use these messages.
- Messages are installed and available after building the workspace.

## Build
```
colcon build --packages-select pingdsp_msg
source install/setup.bash
```
