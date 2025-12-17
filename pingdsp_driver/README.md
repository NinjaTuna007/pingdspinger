
# pingdsp_driver

ROS 2 driver for the PingDSP 3DSS-DX sonar (TCP streaming).

## Features
- TCP client for 3DSS-DX data
- Point cloud filtering and recording
- ROS 2 bag recording support
- PCAP replay for offline testing

## Usage
### Build and Source
```
colcon build --packages-select pingdsp_driver
source install/setup.bash
```

### Launch with PCAP Replay
```
ros2 launch pingdsp_driver test_driver.launch.py
```

#### Arguments
- `replay_speed`: Speed multiplier for replay (default: 50.0)
- `enable_filter`: Enable point cloud filtering (default: true)
- `record_pointcloud`: Enable PLY recording (default: false)
- `record_bag`: Enable ROS 2 bag recording (default: false)
- `pcap_file`: Path to PCAP file (overrides YAML)

### Configuration
- All config, launch, and script files are installed with the package.
- Edit YAML files in `config/` for parameter changes.

### Development
To update installed configs/scripts, rebuild and re-source the workspace.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sonar_host` | string | '192.168.1.100' | IP address of 3DSS-DX sonar (use '127.0.0.1' for pcap replay) |
| `sonar_port` | int | 14001 | TCP port of sonar (use 23848 for pcap replay) |
| `frame_id` | string | 'sonar' | TF frame ID for sensor data |
| `odom_frame_id` | string | 'odom' | TF frame ID for odometry (origin set at first ping) |
| `map_frame_id` | string | 'map' | TF frame ID for map (reserved for future use) |
| `reconnect_delay` | float | 5.0 | Delay between reconnection attempts (seconds) |
| `publish_sidescan` | bool | true | Whether to publish sidescan waterfall imagery |
| `waterfall_max_height` | int | 500 | Maximum number of pings in waterfall buffer |
| `transducer_tilt_deg` | float | -20.0 | Downward tilt angle of transducer in degrees |
|-----------|------|---------|-------------|
| `sonar_host` | string | '192.168.1.100' | IP address of 3DSS-DX sonar |
| `sonar_port` | int | 14001 | TCP port of sonar |
| `frame_id` | string | 'sonar' | TF frame ID for sensor data |
| `map_frame_id` | string | 'map' | TF frame ID for map |
| `reconnect_delay` | float | 5.0 | Delay between reconnection attempts (seconds) |
| `publish_sidescan` | bool | true | Whether to publish sidescan waterfall |
## Usage

### Live Sonar Connection

```bash
# Edit config/3dss_params.yaml to set your sonar's IP address
ros2 launch pingdspinger 3dss.launch
```

### Pcap Replay (Testing without Hardware)

```bash
# Terminal 1: Start the replay server and driver together
ros2 launch pingdspinger test_driver.launch.py

# Optional: Adjust replay speed (default 10x)
ros2 launch pingdspinger test_driver.launch.py replay_speed:=50.0

# Optional: Use custom pcap file
ros2 launch pingdspinger test_driver.launch.py pcap_file:=/path/to/custom.pcap
```

**Pcap Replay Features:**
- Preserves original packet timing with configurable speed multiplier
- Automatically starts TCP server (port 23848) and driver
- Streams packets directly without loading entire file into memory
- Graceful shutdown when replay completes

### Manual Pcap Replay (Advanced)

```bash
# Terminal 1: Start replay server manually
python3 scripts/replay_pcap.py network_dump/pingDSP_traffic.pcap --speed 10

# Terminal 2: Start driver (edit config to use 127.0.0.1:23848)
ros2 launch pingdspinger 3dss.launch
```

### Custom Configuration

```bash
ros2 launch pingdspinger 3dss.launch config_file:=/path/to/custom_params.yaml
```

### Visualization with Foxglove

```bash
# Terminal 1: Start the driver
## TCP Protocol
ros2 launch pingdspinger 3dss.launch
# Terminal 2: Start Foxglove Studio
# You will need to open Foxglove Studio separately and connect to the ROS 2 bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## TCP Protocol

The driver implements the two-step read protocol from the C++ API:

1. **Read DxHeader** (20 bytes):
   - 16-byte preamble: `0x50494E4727 2B3AD8742A 1C33E9B073 B1` ("PING..." signature)
   - 4-byte data_count: length of DxData to follow

2. **Read DxData** (variable length):
   - 88-byte header with ping metadata (ping number, timestamp, counters, rates)
   - Variable-length data sections:
     - Port/starboard bathymetry points (range, angle, amplitude)
     - Port/starboard sidescan samples (uint16 intensity arrays)
pingdspinger/

## Architecture

```
pingdsp_driver/
├── pingdsp_driver/
│   ├── __init__.py
│   ├── dx_structures.py         # Data structures (DxHeader, DxData, etc.)
│   ├── pointcloud_filter.py     # Point cloud filtering node
│   ├── pointcloud_recorder.py   # Point cloud recording node
│   ├── tcp_client.py            # TCP connection manager
│   └── tdss_driver.py           # Main ROS 2 node (TdssDxDriver)
├── config/
│   ├── 3dss_params.yaml         # Driver parameters
│   ├── filter_params.yaml       # Filter parameters
│   ├── paths.yaml               # Paths for data locations
│   └── recorder_params.yaml     # Recorder parameters
├── launch/
│   ├── 3dss.launch              # Launch file for live sonar
│   └── test_driver.launch.py    # Launch file for pcap replay
├── scripts/
│   ├── replay_pcap.py           # TCP replay server for testing
│   └── test_parser.py           # Parser testing utility
├── resource/
│   └── pingdsp_driver           # Package resource marker
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── README.md
├── setup.cfg
├── setup.py
└── LICENSE
```

## Odometry Integration

The driver provides full 6DOF odometry by parsing navigation sensors embedded in the sonar data stream:

### Position Sources (Priority Order)
1. **GPGGA** - GPS position (latitude, longitude) converted to local XY coordinates
2. Ship coordinates from DxData structure (if available)

### Orientation Sources (Priority Order)
1. **VNYCM** - Internal MRU providing yaw, pitch, and roll (preferred)
## Troubleshooting

### Cannot connect to sonar

```bash
# Check network connectivity
ping 192.168.1.100

# Verify port is open (default: 14001)
nc -zv 192.168.1.100 14001

# Check current parameters
ros2 param list /tdss_driver
ros2 param get /tdss_driver sonar_host
```

### No data published

- Check that sonar is transmitting (verify with manufacturer's software)
- Monitor status topic: `ros2 topic echo /sonar/status`
- Verify active topics: `ros2 topic list`
- Check node logs: `ros2 node info /tdss_driver`

## Testing

### Verify Installation

```bash
# Build package
cd ~/colcon_ws
colcon build --packages-select pingdspinger
source install/setup.bash

# Test with pcap replay
ros2 launch pingdspinger test_driver.launch.py

# Check topics are publishing
ros2 topic list
ros2 topic hz /sonar/bathymetry
ros2 topic echo /sonar/status --no-arr
```

### Monitor Data Flow

```bash
# Watch bathymetry point cloud rate
ros2 topic hz /sonar/bathymetry

# View raw NMEA/TSS1 sentences
ros2 topic echo /sonar/nmea

# Monitor vehicle trajectory
ros2 topic echo /vehicle/path --no-arr

# Check TF tree
ros2 run tf2_tools view_frames
```

## References

- **C++ API**: pingdsp-3dss.hpp v0.6 (2016-11-09, PingDSP Inc.)
- **TCP Protocol**: Default port 14001, binary stream with 16-byte preamble signature
- **Network Replay**: Uses tshark for pcap parsing and socket streaming
- **Companion Package**: `pingdsp_xtf_visualizer` (for XTF file playback)

## License

Apache-2.0

## Author

ROS 2 implementation based on PingDSP C++ API documentation and reverse-engineered TCP protocol.
# Verify pcap file has TCP data on expected port
tshark -r network_dump/pingDSP_traffic.pcap -Y "tcp.srcport == 23848" | head

# Check replay server is listening
netstat -tuln | grep 23848

# Test with slower replay speed
ros2 launch pingdspinger test_driver.launch.py replay_speed:=1.0
```

### Float conversion errors in Ping3DSS message

- Fixed in latest version: sidescan samples now explicitly converted to uint16
- If error persists, check that message definitions are rebuilt:
  ```bash
  colcon build --packages-select pingdspinger --cmake-clean-cache
  ```
- **TdssDxDriver**: Main node handling TCP connection, data parsing, and ROS publishing
- **TcpClient**: Manages socket connection with automatic resynchronization on stream corruption
- **DxData**: Parses binary sonar data structures following pingdsp-3dss.hpp v0.6 API
- **replay_pcap.py**: Extracts TCP payloads from pcap and streams to local socket with timing control
### Build

```bash
cd ~/colcon_ws
colcon build --packages-select pingdspinger
source install/setup.bash
```

## TCP Protocol

The driver implements the two-step read protocol from the C++ API:

1. **Read DxHeader** (20 bytes):
   - Preamble: `0x5844444C` ("XDDL")
   - Data length field
   - 3 reserved uint32 fields

2. **Read DxData** (variable length):
   - 88-byte header with ping metadata
   - Data sections: bathymetry, sidescan, 3D points, ASCII data

## Architecture

```
pingdspinger/
├── pingdspinger/
│   ├── __init__.py
│   ├── 3dss_driver.py      # Main ROS 2 node
│   ├── tcp_client.py        # TCP connection manager
│   └── dx_structures.py     # Data structures (DxHeader, DxData, etc.)
├── launch/
│   └── 3dss.launch          # Launch file
├── config/
│   └── 3dss_params.yaml     # Parameters
├── msg/
│   ├── Ping3DSS.msg         # Custom ping message
│   ├── SonarSettings.msg    # Sonar settings message
│   └── SystemInfo.msg       # System info message
├── package.xml
├── CMakeLists.txt
├── setup.py
└── README.md
```

## Comparison with pingdsp_xtf_visualizer

| Feature | pingdspinger | pingdsp_xtf_visualizer |
|---------|-----------------|------------------------|
| **Data Source** | Live TCP stream | Recorded XTF files |
| **Use Case** | Real-time operations | Post-mission analysis |
| **Format** | C++ API (pingdsp-3dss.hpp) | XTF file format (CAATI) |
| **Connection** | TCP socket to sonar | File I/O |
| **Replay Control** | N/A (real-time) | Speed control, pause, seek |

## Troubleshooting

### Cannot connect to sonar

```bash
# Check network connectivity
ping 192.168.1.100

# Verify port (default: 14001)
nc -zv 192.168.1.100 14001
```

### No data published

- Check that sonar is transmitting (verify with manufacturer's software)
- Monitor status topic: `ros2 topic echo /sonar/status`
- Check logs: `ros2 node logs /tdss_driver`

### Invalid preamble errors

- Ensure sonar is using DX protocol (not legacy format)
- Check C++ API version compatibility (tested with v0.6)

## References

- **C++ API**: pingdsp-3dss.hpp v0.6 (PingDSP documentation)
- **TCP Protocol**: Default port 14001, binary data stream
- **Companion Package**: `pingdsp_xtf_visualizer` (for XTF file playback)

## License

Apache-2.0

## Author

ROS 2 implementation based on PingDSP C++ API documentation.
