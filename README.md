


# pingdspinger: ROS 2 Driver & Tools for PingDSP 3DSS-DX Sonar

This repository provides ROS 2 packages, drivers, message definitions, and example data for the PingDSP 3DSS-DX sonar. It enables TCP streaming, replay from PCAP, and integration with ROS 2 workflows.


## Repository Structure

- `pingdsp_driver/` — Main ROS 2 driver for PingDSP 3DSS-DX sonar (TCP streaming)
- `pingdsp_msg/` — Custom ROS 2 message definitions for PingDSP data
- `bags/` — Example MCAP/ROS 2 bag files for replay/testing
- `network_dump/` — Example PCAP files for replay/testing


## Quick Start

```bash
colcon build
source install/setup.bash
```


### Run the Driver with PCAP Replay

```bash
ros2 launch pingdsp_driver test_driver.launch.py
```


### Run with Live Sonar

Edit `config/3dss_params.yaml` to set your sonar's IP address, then:

```bash
ros2 launch pingdsp_driver 3dss.launch
```


## Usage

### PCAP Replay Options

- Adjust replay speed:
   ```bash
   ros2 launch pingdsp_driver test_driver.launch.py replay_speed:=50.0
   ```
- Use a custom PCAP file:
   ```bash
   ros2 launch pingdsp_driver test_driver.launch.py pcap_file:=/path/to/file.pcap
   ```

### Manual PCAP Replay (Advanced)

```bash
# Terminal 1: Start replay server manually
python3 scripts/replay_pcap.py network_dump/pingDSP_traffic.pcap --speed 10

# Terminal 2: Start driver (edit config to use 127.0.0.1:23848)
ros2 launch pingdsp_driver 3dss.launch
```

### Custom Configuration

```bash
ros2 launch pingdsp_driver 3dss.launch config_file:=/path/to/custom_params.yaml
```


## Directory Layout

```text
pingdspinger/
├── pingdsp_driver/         # Main ROS 2 driver package
├── pingdsp_msg/            # Custom message definitions
├── bags/                   # Example bag files
├── network_dump/           # Example PCAP files
```


## Troubleshooting

### Cannot Connect to Sonar

```bash
ping 192.168.1.100
nc -zv 192.168.1.100 14001
ros2 param get /tdss_driver sonar_host
```

### No Data Published

- Ensure the sonar is transmitting (check with manufacturer software)
- Monitor status topic: `ros2 topic echo /sonar/status`
- Check node logs: `ros2 node info /tdss_driver`

### PCAP Replay Not Working

- Ensure the PCAP file contains TCP data on the expected port (default: 23848):
   ```bash
   tshark -r network_dump/pingDSP_traffic.pcap -Y "tcp.srcport == 23848" | head
   ```
- Check that the replay server is listening:
   ```bash
   netstat -tuln | grep 23848
   ```


## License

Apache-2.0

## Author

Shekhar Devm Upadhyay, based on PingDSP C++ API documentation.
