# Complete 3DSS-DX ping message with all metadata

std_msgs/Header header

# Ping identification
uint64 ping_number
uint32 milliseconds_today

# Position (from ship coordinates in DxData)
float32 ship_x_coordinate
float32 ship_y_coordinate

# System information
string sonar_id
float32 acoustic_frequency
float32 sample_rate
float32 maximum_ping_rate
float32 port_transducer_angle
float32 starboard_transducer_angle

# Sonar settings
float32 range_setting
float32 sound_velocity_bulk
float32 sound_velocity_face
float32 transmit_angle
uint32 transmit_power

# Port range resolutions
float32 port_sidescan_range_resolution
float32 port_sidescan3d_range_resolution

# Starboard range resolutions
float32 starboard_sidescan_range_resolution
float32 starboard_sidescan3d_range_resolution

# Bathymetry data
sensor_msgs/PointCloud2 port_bathymetry
sensor_msgs/PointCloud2 starboard_bathymetry

# Sidescan data (raw samples)
uint16[] port_sidescan_samples
uint16[] starboard_sidescan_samples

# ASCII data (NMEA, TSS1)
string ascii_data

# Recorded file information (if from recorded data)
string recorded_filename
string recorded_version
