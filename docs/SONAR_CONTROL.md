# Sonar Control Interface

ROS 2 service-based control interface for PingDSP 3DSS-DX sonar settings.

## Overview

The sonar control system provides ROS 2 services to dynamically adjust sonar parameters during operation. This is based on the **3DSS-DX Control Command Interface Guide 1.4**.

## Architecture

```
┌─────────────────────────────────────────────────┐
│  sonar_control_node                             │
│  ┌──────────────────────────────────────────┐  │
│  │  TCP Control Connection (port 23840)     │  │
│  │  - Send ASCII commands                   │  │
│  │  - Parse responses                       │  │
│  └──────────────────────────────────────────┘  │
│                                                 │
│  Services:                                      │
│  • /sonar/set_range                             │
│  • /sonar/set_gain                              │
│  • /sonar/set_power                             │
│  • /sonar/set_sound_velocity                    │
│  • /sonar/get_settings                          │
│  • /sonar/set_trigger_mode                      │
└─────────────────────────────────────────────────┘
```

## Available Services

### 1. Set Range
Adjust the sonar operating range in meters.

```bash
ros2 service call /sonar/set_range pingdsp_msg/srv/SetSonarRange "{range: 50.0}"
```

### 2. Set Gain (TVG)
Configure Time-Varied Gain for port or starboard side.

```bash
ros2 service call /sonar/set_gain pingdsp_msg/srv/SetSonarGain \
  "{side: 'port', constant: 10.0, linear: 0.5, logarithmic: 20.0}"
```

### 3. Set Transmit Power
Adjust transmit power for port or starboard side.

```bash
ros2 service call /sonar/set_power pingdsp_msg/srv/SetSonarPower \
  "{side: 'starboard', power: 75}"
```

### 4. Set Sound Velocity
Update sound velocity for range calculations (typically 1450-1550 m/s).

```bash
ros2 service call /sonar/set_sound_velocity pingdsp_msg/srv/SetSoundVelocity \
  "{sound_velocity: 1500.0}"
```

### 5. Get Current Settings
Query current sonar configuration.

```bash
ros2 service call /sonar/get_settings pingdsp_msg/srv/GetSonarSettings
```

### 6. Set Trigger Mode
Configure external or continuous triggering.

```bash
# External trigger
ros2 service call /sonar/set_trigger_mode pingdsp_msg/srv/SetTriggerMode \
  "{trigger_source: 'external', duty_cycle: 0.0}"

# Continuous trigger with duty cycle
ros2 service call /sonar/set_trigger_mode pingdsp_msg/srv/SetTriggerMode \
  "{trigger_source: 'continuous', duty_cycle: 0.8}"
```

## Launch Configuration

Enable/disable the control node in the launch file:

```bash
# With control enabled (default)
ros2 launch pingdsp_driver 3dss.launch enable_control:=true

# Without control
ros2 launch pingdsp_driver 3dss.launch enable_control:=false
```

## Configuration Parameters

Edit [config/control_params.yaml](config/control_params.yaml):

```yaml
sonar_control_node:
  ros__parameters:
    sonar_host: '192.168.228.50'  # Sonar IP address
    control_port: 23840            # Control port for PingDSP 3DSS-DX
    timeout: 5.0                   # Connection timeout
    reconnect_on_disconnect: true  # Auto-reconnect on connection loss
```

## Python Example

```python
import rclpy
from rclpy.node import Node
from pingdsp_msg.srv import SetSonarRange, SetSonarGain

class SonarController(Node):
    def __init__(self):
        super().__init__('sonar_controller')
        self.range_client = self.create_client(SetSonarRange, '/sonar/set_range')
        
    def set_range(self, range_m):
        request = SetSonarRange.Request()
        request.range = range_m
        
        future = self.range_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info(f'Range set to {range_m}m')
        else:
            self.get_logger().error(f'Failed: {future.result().message}')

def main():
    rclpy.init()
    controller = SonarController()
    controller.set_range(75.0)
    rclpy.shutdown()
```

## Building

```bash
cd ~/colcon_ws
colman build --packages-select pingdsp_msg pingdsp_driver
source install/setup.bash
```

## Testing Services

List all available services:
```bash
ros2 service list | grep sonar
```

Get service type information:
```bash
ros2 service type /sonar/set_range
```

View service interface:
```bash
ros2 interface show pingdsp_msg/srv/SetSonarRange
```

## Troubleshooting

### Connection Issues
- Verify sonar IP address in `control_params.yaml`
- Check control port (default 23840 for PingDSP 3DSS-DX)
- Ensure control interface is enabled on sonar hardware
- Check firewall settings

### Command Failures
- Review node logs: `ros2 node info /sonar_control_node`
- Check response messages in service calls
- Verify command syntax matches 3DSS-DX Control Interface Guide

### Service Not Available
```bash
# Check if node is running
ros2 node list | grep sonar_control

# Restart control node
ros2 run pingdsp_driver sonar_control_node
```

## Future Enhancements

- **rqt GUI Plugin**: Graphical interface for operators
- **Settings Persistence**: Save/load configuration profiles
- **Automatic Tuning**: Adaptive gain based on depth/conditions
- **Batch Commands**: Send multiple settings atomically

## Related Documentation

- **3DSS-DX Control Command Interface Guide 1.4**: Full command reference
- **ROS 2 Services Tutorial**: https://docs.ros.org/en/humble/Tutorials/Services.html
- **PingDSP Driver README**: Main driver documentation
