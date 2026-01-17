# System Architecture Diagram

## Complete PingDSP ROS 2 Integration

```
┌─────────────────────────────────────────────────────────────────────┐
│                        PingDSP 3DSS-DX Sonar                        │
│                                                                     │
│  ┌──────────────────────┐        ┌──────────────────────┐          │
│  │  Data Stream Port    │        │  Control Port        │          │
│  │  (TCP 14001)         │        │  (TCP 23840)         │          │
│  └──────────┬───────────┘        └──────────┬───────────┘          │
└─────────────┼────────────────────────────────┼──────────────────────┘
              │                                │
              │ Binary data stream             │ ASCII commands
              │ (Ping3DSS packets)             │ (SET/GET commands)
              │                                │
┌─────────────▼────────────────┐  ┌───────────▼──────────────────────┐
│  tdss_driver                 │  │  sonar_control_node (NEW!)       │
│  (Existing)                  │  │                                  │
│                              │  │  ROS 2 Services:                 │
│  Publishers:                 │  │  ┌────────────────────────────┐  │
│  • /sonar/bathymetry        │  │  │ /sonar/set_range           │  │
│    (PointCloud2)             │  │  │ /sonar/set_gain            │  │
│  • /sonar/ping               │  │  │ /sonar/set_power           │  │
│    (Ping3DSS)                │  │  │ /sonar/set_sound_velocity  │  │
│  • /sonar/intensity          │  │  │ /sonar/get_settings        │  │
│    (Image)                   │  │  │ /sonar/set_trigger_mode    │  │
│  • /sonar/pose               │  │  └────────────────────────────┘  │
│    (PoseStamped)             │  │                                  │
│  • /sonar/nmea               │  │  Control Interface:              │
│    (String)                  │  │  • Sends ASCII commands          │
│  • /sonar/status             │  │  • Parses responses              │
│    (String)                  │  │  • Auto-reconnect                │
└──────────────────────────────┘  └──────────────────────────────────┘
              │                                │
              │                                │
              └────────────┬───────────────────┘
                           │
              ┌────────────▼────────────┐
              │  ROS 2 Ecosystem        │
              │                         │
              │  • rviz2                │
              │  • rosbag               │
              │  • Custom nodes         │
              │  • rqt (future GUI)     │
              │  • Scripts              │
              └─────────────────────────┘

* Data port 14001 is for binary ping data stream (DxData protocol)
* Control port 23840 is for ASCII command interface
```

## Data Flow

### Sonar Data (Continuous Stream)
```
Sonar Hardware
    │
    ├─► [TCP 14001] Binary Ping Packets
    │
    └─► tdss_driver
            │
            ├─► /sonar/bathymetry (PointCloud2)
            ├─► /sonar/intensity (Image)
            ├─► /sonar/ping (Ping3DSS)
            └─► /sonar/pose (PoseStamped)
```

### Sonar Control (Request/Response)
```
User/Script
    │
    └─► Service Call: ros2 service call /sonar/set_range {...}
            │
            └─► sonar_control_node
                    │
                    ├─► [TCP 23840] ASCII: "SET RANGE 50"
                    │
                    └─► Sonar Hardware
                            │
                            └─► Response: "OK" / "ERROR"
                                    │
                                    └─► Service Response
```

## Integration Points

### 1. Command Line
```bash
ros2 service call /sonar/set_range \
  pingdsp_msg/srv/SetSonarRange "{range: 75.0}"
```

### 2. Python Node
```python
client = node.create_client(SetSonarRange, '/sonar/set_range')
request = SetSonarRange.Request()
request.range = 75.0
future = client.call_async(request)
```

### 3. Launch File
```xml
<node pkg="pingdsp_driver" exec="sonar_control_node" 
      name="sonar_control_node" if="$(var enable_control)">
  <param from="$(var control_config)"/>
</node>
```

### 4. Future: rqt GUI
```
┌────────────────────────────────┐
│  Sonar Control Panel (rqt)     │
│                                │
│  Range: [====|=====] 75m       │
│  Gain:  [===|======] 15        │
│  Power: [======|===] 80        │
│                                │
│  [Apply Settings]              │
└────────────────────────────────┘
         │
         └─► Calls services internally
```

## Deployment Scenarios

### Scenario 1: Full System
```bash
ros2 launch pingdsp_driver 3dss.launch \
  enable_filter:=true \
  enable_recorder:=false \
  enable_control:=true
```
- Data streaming: ✓
- Point cloud filtering: ✓
- Recording: ✗
- Control interface: ✓

### Scenario 2: Data Only (No Control)
```bash
ros2 launch pingdsp_driver 3dss.launch \
  enable_control:=false
```
- Data streaming: ✓
- Control interface: ✗

### Scenario 3: Replay + Control
```bash
# Terminal 1: Replay PCAP
python3 scripts/replay_pcap.py network_dump/file.pcap

# Terminal 2: Driver + Control
ros2 launch pingdsp_driver 3dss.launch enable_control:=true
```
- Data from replay: ✓
- Control interface: ✓ (to real hardware)

## Service Call Examples

### Range Adjustment
```bash
# Short range (harbor)
ros2 service call /sonar/set_range pingdsp_msg/srv/SetSonarRange "{range: 25.0}"

# Medium range (survey)
ros2 service call /sonar/set_range pingdsp_msg/srv/SetSonarRange "{range: 75.0}"

# Long range (search)
ros2 service call /sonar/set_range pingdsp_msg/srv/SetSonarRange "{range: 150.0}"
```

### Environment Adaptation
```bash
# Freshwater lake
ros2 service call /sonar/set_sound_velocity \
  pingdsp_msg/srv/SetSoundVelocity "{sound_velocity: 1482.0}"

# Seawater (20°C)
ros2 service call /sonar/set_sound_velocity \
  pingdsp_msg/srv/SetSoundVelocity "{sound_velocity: 1520.0}"
```

### Gain Tuning
```bash
# Low gain (clear water, strong targets)
ros2 service call /sonar/set_gain pingdsp_msg/srv/SetSonarGain \
  "{side: 'port', constant: 5.0, linear: 0.3, logarithmic: 15.0}"

# High gain (turbid water, weak targets)
ros2 service call /sonar/set_gain pingdsp_msg/srv/SetSonarGain \
  "{side: 'port', constant: 20.0, linear: 1.5, logarithmic: 35.0}"
```

## Files Created

```
pingdspinger/
├── pingdsp_msg/
│   ├── srv/                           (NEW)
│   │   ├── SetSonarRange.srv          ✓
│   │   ├── SetSonarGain.srv           ✓
│   │   ├── SetSonarPower.srv          ✓
│   │   ├── SetSoundVelocity.srv       ✓
│   │   ├── GetSonarSettings.srv       ✓
│   │   └── SetTriggerMode.srv         ✓
│   └── CMakeLists.txt                 (Modified)
│
├── pingdsp_driver/
│   ├── pingdsp_driver/
│   │   └── sonar_control_node.py      ✓ (NEW)
│   ├── config/
│   │   └── control_params.yaml        ✓ (NEW)
│   ├── launch/
│   │   └── 3dss.launch                (Modified)
│   ├── scripts/
│   │   └── example_sonar_control.py   ✓ (NEW)
│   ├── setup.py                       (Modified)
│   └── SONAR_CONTROL.md               ✓ (NEW)
│
├── README.md                          (Modified)
├── IMPLEMENTATION_SUMMARY.md          ✓ (NEW)
├── QUICKSTART_CHECKLIST.md            ✓ (NEW)
└── ARCHITECTURE.md                    ✓ (THIS FILE)
```

---

**✅ System Ready for Deployment**
