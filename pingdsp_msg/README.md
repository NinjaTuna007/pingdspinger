# pingdsp_msg

Custom ROS 2 message definitions for PingDSP 3DSS-DX sonar data.

## Messages
- `Ping3DSS.msg`: Main sonar ping data (raw sidescan samples + per-ping metadata)
- `SonarSettings.msg`: Every acquisition/processing setting the head reports in
  each ping (range, sample rate, TVG gain polynomial, pulse, beamwidth, power,
  transmit angle, trigger, 2D/3D processing, sound velocity). Published latched on
  `sonar/settings` whenever a value changes.
- `SonarAltitude.msg`: The head's own nadir depth (`$PDNDE`), one per ping on
  `sonar/altitude`. `altitude = -nadir_depth`; `field2..4` are the sentence's
  remaining undocumented fields, carried verbatim.
- `SystemInfo.msg`: System information

## Usage
- Add `pingdsp_msg` as a dependency in your ROS 2 package to use these messages.
- Messages are installed and available after building the workspace.

## Build
```
colcon build --packages-select pingdsp_msg
source install/setup.bash
```
