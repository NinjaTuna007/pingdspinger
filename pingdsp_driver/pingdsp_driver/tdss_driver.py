#!/usr/bin/env python3
"""
ROS 2 Driver for PingDSP 3DSS-DX Sonar

Connects to 3DSS-DX sonar via TCP and publishes:
- PointCloud2: Bathymetry point clouds
- Image: Sidescan waterfall imagery
- NavSatFix: GPS position (from NMEA sentences)
- Imu: Attitude data (from TSS1 sentences)
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField, Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, String
from tf2_ros import TransformBroadcaster

from pingdsp_msg.msg import Ping3DSS, SystemInfo

import numpy as np
import logging
import re
from datetime import datetime
from typing import Optional

from pyproj import Transformer

from pingdsp_driver.tcp_client import TcpClient
from pingdsp_driver.dx_structures import DxData, DX_TCP_PORT


class TdssDxDriver(Node):
    """ROS 2 driver node for 3DSS-DX sonar TCP streaming."""
    
    def __init__(self):
        super().__init__('tdss_dx_driver')
        
        # Declare parameters
        self.declare_parameter('sonar_host', '192.168.1.100')
        self.declare_parameter('sonar_port', DX_TCP_PORT)
        self.declare_parameter('frame_id', 'sonar')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('reconnect_delay', 5.0)
        self.declare_parameter('publish_sidescan', True)
        self.declare_parameter('waterfall_max_height', 10000)
        self.declare_parameter('waterfall_target_width', 2000)
        self.declare_parameter('transducer_tilt_deg', -20.0)  # Downward tilt angle
        
        # Get parameters
        self.sonar_host = self.get_parameter('sonar_host').value
        self.sonar_port = self.get_parameter('sonar_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.reconnect_delay = self.get_parameter('reconnect_delay').value
        self.publish_sidescan = self.get_parameter('publish_sidescan').value
        self.waterfall_max_height = self.get_parameter('waterfall_max_height').value
        self.waterfall_target_width = self.get_parameter('waterfall_target_width').value
        self.transducer_tilt_deg = self.get_parameter('transducer_tilt_deg').value
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = self.get_logger()
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            'sonar/bathymetry',
            sensor_qos
        )
        
        self.ping_pub = self.create_publisher(
            Ping3DSS,
            'sonar/ping',
            10
        )
        
        self.intensity_pub = self.create_publisher(
            Image,
            'sonar/intensity',
            sensor_qos
        )
        
        self.pose_pub = self.create_publisher(
            PoseStamped,
            'sonar/pose',
            10
        )
        
        self.vehicle_pos_pub = self.create_publisher(
            PoseStamped,
            'vehicle/position',
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            'vehicle/path',
            10
        )
        
        self.ascii_pub = self.create_publisher(
            String,
            'sonar/nmea',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            'sonar/status',
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State
        self.tcp_client: Optional[TcpClient] = None
        self.connected = False
        self.ping_count = 0
        
        # Waterfall display buffer - pre-allocated for speed
        self.waterfall_buffer = None  # Will be initialized on first ping
        self.waterfall_width = 0  # Track expected width
        self.waterfall_count = 0  # Number of pings added to buffer
        self.waterfall_filled = False  # Whether buffer is full yet
        
        # Odom frame initialization (full 6DOF)
        self.odom_origin_set = False
        self.odom_origin_x = 0.0
        self.odom_origin_y = 0.0
        self.odom_origin_z = 0.0
        self.odom_origin_roll = 0.0
        self.odom_origin_pitch = 0.0
        self.odom_origin_yaw = 0.0
        
        # Current pose (position + orientation)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0  # 0° = level/horizontal
        self.current_yaw = 0.0
        
        # Connection retry tracking
        self.connection_lost_count = 0
        self.max_connection_retries = 20  # Exit after 20 failed reconnection attempts (2 seconds)
        
        # UTM coordinate transformer (initialized on first GPS fix)
        self.utm_transformer = None
        self.utm_zone = None
        self.utm_meridian_convergence = 0.0  # Grid convergence angle in radians
        
        # Vehicle trajectory path
        self.vehicle_path = Path()
        self.vehicle_path.header.frame_id = self.odom_frame_id
        
        # Create timer for connection attempts and data reading
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.logger.info(f"3DSS-DX Driver initialized")
        self.logger.info(f"  Target: {self.sonar_host}:{self.sonar_port}")
        self.logger.info(f"  Frame ID: {self.frame_id}")
        self.logger.info(f"  Odom Frame: {self.odom_frame_id}")
        self.publish_status(f"Initializing connection to {self.sonar_host}:{self.sonar_port}")
    
    def publish_status(self, message: str):
        """Publish status message."""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
    
    def connect_to_sonar(self) -> bool:
        """Attempt to connect to 3DSS-DX sonar."""
        if self.connected:
            return True
        
        try:
            self.logger.info(f"Connecting to {self.sonar_host}:{self.sonar_port}...")
            self.publish_status(f"Connecting to {self.sonar_host}:{self.sonar_port}...")
            
            self.tcp_client = TcpClient(self.sonar_host, self.sonar_port)
            if self.tcp_client.connect():
                self.connected = True
                self.logger.info("Connected successfully!")
                self.publish_status("Connected to 3DSS-DX sonar")
                return True
            else:
                self.logger.warning("Connection failed")
                self.publish_status("Connection failed, will retry...")
                return False
        
        except Exception as e:
            self.logger.error(f"Connection error: {e}")
            self.publish_status(f"Connection error: {e}")
            return False
    
    def timer_callback(self):
        """Main timer callback - connects and reads data."""
        # Ensure connection
        if not self.connected:
            if not self.connect_to_sonar():
                self.connection_lost_count += 1
                if self.connection_lost_count >= self.max_connection_retries:
                    self.logger.info("Unable to reconnect after multiple attempts. Shutting down.")
                    self.publish_status("Connection lost, shutting down...")
                    raise SystemExit(0)  # Clean exit
                return  # Will retry on next timer callback
            else:
                # Successfully reconnected, reset counter
                self.connection_lost_count = 0
        
        # Read one ping from TCP stream
        try:
            result = self.tcp_client.read_ping()
            if result is None:
                # Connection lost or timeout
                if not self.tcp_client.connected:
                    self.logger.warning("Connection lost, reconnecting...")
                    self.publish_status("Connection lost, reconnecting...")
                    self.connected = False
                return
            
            header, data = result
            self.process_ping(data)
        
        except Exception as e:
            self.logger.error(f"Error reading ping: {e}")
            self.connected = False
    
    def process_ping(self, data: DxData):
        """Process a received ping and publish ROS messages."""
        self.ping_count += 1
        
        # Create timestamp
        timestamp = self.create_timestamp(data.milliseconds_today)
        
        # Extract position from ASCII data if available (NMEA sentences)
        self.update_position_from_data(data)
        
        # Set odom origin on first ping (full 6DOF: position + orientation)
        if not self.odom_origin_set:
            self.odom_origin_set = True
            self.odom_origin_x = self.current_x
            self.odom_origin_y = self.current_y
            self.odom_origin_z = self.current_z
            self.odom_origin_roll = self.current_roll
            self.odom_origin_pitch = self.current_pitch
            self.odom_origin_yaw = self.current_yaw
            self.logger.info(f"Odom origin set at pos=({self.odom_origin_x:.2f}, {self.odom_origin_y:.2f}, {self.odom_origin_z:.2f}) "
                           f"rot=({np.rad2deg(self.odom_origin_roll):.1f}°, {np.rad2deg(self.odom_origin_pitch):.1f}°, {np.rad2deg(self.odom_origin_yaw):.1f}°)")
        
        # Publish TF tree: map -> odom -> sonar
        self.publish_map_to_odom_tf(timestamp)
        self.publish_tf(timestamp)
        
        # Publish vehicle position in odom frame
        if self.vehicle_pos_pub.get_subscription_count() > 0 or self.path_pub.get_subscription_count() > 0:
            self.publish_vehicle_position(timestamp)
        
        # Publish complete ping message with all metadata
        if self.ping_pub.get_subscription_count() > 0:
            self.publish_full_ping(data, timestamp)
        
        # Publish bathymetry point cloud
        if self.pointcloud_pub.get_subscription_count() > 0:
            self.publish_bathymetry(data, timestamp)
        
        # Publish sidescan waterfall
        if self.publish_sidescan and self.intensity_pub.get_subscription_count() > 0:
            self.publish_sidescan_waterfall(data, timestamp)
        
        # Publish ASCII data (NMEA, TSS1)
        if self.ascii_pub.get_subscription_count() > 0:
            ascii_data = data.get_ascii_data()
            if ascii_data:
                msg = String()
                msg.data = ascii_data
                self.ascii_pub.publish(msg)
        
        # Log progress
        if self.ping_count % 10 == 0:
            self.logger.info(f"Processed {self.ping_count} pings (ping#{data.ping_number}, "
                           f"{data.port_bathy_count + data.stbd_bathy_count} bathy points)")
    
    def create_timestamp(self, milliseconds_today: int) -> Time:
        """Create ROS timestamp - uses current time for live playback."""
        # Use current ROS time instead of sonar's internal timestamp
        # This ensures TF and messages appear "live" in visualization tools
        return self.get_clock().now()
    
    def publish_full_ping(self, data: DxData, timestamp: Time):
        """Publish complete Ping3DSS message with all metadata."""
        try:
            msg = Ping3DSS()
            msg.header = Header()
            msg.header.stamp = timestamp.to_msg()
            msg.header.frame_id = self.frame_id
            
            # Ping identification
            msg.ping_number = int(data.ping_number)
            msg.milliseconds_today = int(data.milliseconds_today)
            
            # Ship coordinates (UTM easting/northing)
            msg.ship_x_coordinate = float(self.current_x)
            msg.ship_y_coordinate = float(self.current_y)
            
            # System info (would need to be parsed from DxData)
            msg.sonar_id = data.system_info.sonar_id
            msg.acoustic_frequency = float(data.system_info.acoustic_frequency)
            msg.sample_rate = float(data.system_info.sample_rate)
            msg.maximum_ping_rate = float(data.system_info.maximum_ping_rate)
            msg.port_transducer_angle = float(data.system_info.port_transducer_angle)
            msg.starboard_transducer_angle = float(data.system_info.starboard_transducer_angle)
            
            # Sonar settings (would need to be parsed from DxParameters)
            msg.range_setting = float(data.parameters.range_m)
            msg.sound_velocity_bulk = float(data.parameters.sound_velocity.bulk)
            msg.sound_velocity_face = float(data.parameters.sound_velocity.face)
            msg.transmit_angle = float(data.parameters.port_transmit.angle)  # Using port, could average both
            msg.transmit_power = int(data.parameters.port_transmit.power)
            
            # Range resolutions
            msg.port_sidescan_range_resolution = float(data.system_info.port_sidescan_range_resolution)
            msg.port_sidescan3d_range_resolution = float(data.system_info.port_sidescan3d_range_resolution)
            msg.starboard_sidescan_range_resolution = float(data.system_info.starboard_sidescan_range_resolution)
            msg.starboard_sidescan3d_range_resolution = float(data.system_info.starboard_sidescan3d_range_resolution)
            
            # Bathymetry point clouds
            port_points = data.get_port_bathymetry()
            stbd_points = data.get_stbd_bathymetry()
            
            if len(port_points) > 0:
                port_xyz = np.array([p.to_xyz() for p in port_points], dtype=np.float32)
                port_amplitudes = np.array([p.amplitude for p in port_points], dtype=np.float32).reshape(-1, 1)
                port_data = np.column_stack([port_xyz, port_amplitudes])
                msg.port_bathymetry = self.create_pointcloud2(port_data, timestamp)
            
            if len(stbd_points) > 0:
                stbd_xyz = np.array([p.to_xyz() for p in stbd_points], dtype=np.float32)
                stbd_amplitudes = np.array([p.amplitude for p in stbd_points], dtype=np.float32).reshape(-1, 1)
                stbd_data = np.column_stack([stbd_xyz, stbd_amplitudes])
                msg.starboard_bathymetry = self.create_pointcloud2(stbd_data, timestamp)
            
            # Sidescan samples (convert to uint16)
            port_sidescan = data.get_port_sidescan()
            stbd_sidescan = data.get_stbd_sidescan()
            msg.port_sidescan_samples = port_sidescan.astype(np.uint16).tolist() if len(port_sidescan) > 0 else []
            msg.starboard_sidescan_samples = stbd_sidescan.astype(np.uint16).tolist() if len(stbd_sidescan) > 0 else []
            
            # ASCII data
            msg.ascii_data = data.get_ascii_data()
            
            # Recorded file info
            msg.recorded_filename = data.get_recorded_filename()
            msg.recorded_version = data.get_recorded_version()
            
            self.ping_pub.publish(msg)
        
        except Exception as e:
            import traceback
            self.logger.error(f"Error publishing full ping message: {e}")
            self.logger.error(f"Traceback: {traceback.format_exc()}")
            self.logger.error(f"Data types - ping_number: {type(data.ping_number)}, milliseconds_today: {type(data.milliseconds_today)}, sample_rate_hz: {type(data.sample_rate_hz)}, ping_rate_hz: {type(data.ping_rate_hz)}")
    
    def publish_bathymetry(self, data: DxData, timestamp: Time):
        """Publish bathymetry point cloud in sonar frame."""
        try:
            # Get XYZ coordinates in sonar frame
            points_sonar = data.get_all_bathymetry_xyz(self.transducer_tilt_deg)
            
            if len(points_sonar) == 0:
                return
            
            # Get amplitude values
            port_points = data.get_port_bathymetry()
            stbd_points = data.get_stbd_bathymetry()
            amplitudes = np.array([p.amplitude for p in port_points + stbd_points], 
                                 dtype=np.float32)
            
            # Replace NaN/inf amplitudes with 0.0
            amplitudes = np.nan_to_num(amplitudes, nan=0.0, posinf=0.0, neginf=0.0)
            
            # Combine xyz + intensity
            points = np.column_stack([points_sonar, amplitudes])
            
            # Create PointCloud2 in sonar frame
            cloud_msg = self.create_pointcloud2(points, timestamp, frame_id=self.frame_id)
            self.pointcloud_pub.publish(cloud_msg)
        
        except Exception as e:
            self.logger.error(f"Error publishing bathymetry: {e}")
    
    def publish_sidescan_waterfall(self, data: DxData, timestamp: Time):
        """
        Publish sidescan intensity waterfall image with bronze colormap.
        
        Creates a scrolling waterfall display where each row represents one ping.
        Port samples are on the left, starboard on the right, with the nadir in the middle.
        Uses simple square root compression and bronze colormap.
        """
        try:
            # Get port and starboard sidescan samples (amplitude values in float32)
            port_samples = data.get_port_sidescan()
            stbd_samples = data.get_stbd_sidescan()
            
            if len(port_samples) == 0 and len(stbd_samples) == 0:
                return
            
            # Concatenate port and starboard with port flipped (left side of image)
            if len(port_samples) > 0 and len(stbd_samples) > 0:
                ping_samples = np.concatenate([np.flip(port_samples), stbd_samples])
            elif len(port_samples) > 0:
                ping_samples = np.flip(port_samples)
            else:
                ping_samples = stbd_samples
            
            # Downsample to target width for visualization
            original_width = len(ping_samples)
            target_width = int(self.waterfall_target_width)
            
            if original_width > target_width:
                # Downsample by averaging bins
                downsample_factor = original_width // target_width
                # Reshape and average
                trim_size = (original_width // downsample_factor) * downsample_factor
                ping_samples = ping_samples[:trim_size].reshape(-1, downsample_factor).mean(axis=1)
            
            # Debug: Log data range for first few pings
            if self.ping_count <= 3:
                self.logger.info(f"Sidescan samples: {original_width} -> {len(ping_samples)} (downsampled), "
                               f"range=[{ping_samples.min():.6f}, {ping_samples.max():.6f}], "
                               f"mean={ping_samples.mean():.6f}")
            
            # Apply square root compression for better dynamic range
            ping_float = np.abs(ping_samples.astype(np.float32))
            
            # Filter out NaN and inf values BEFORE sqrt to prevent corruption
            if not np.isfinite(ping_float).all():
                nan_count = (~np.isfinite(ping_float)).sum()
                ping_float = np.nan_to_num(ping_float, nan=0.0, posinf=0.0, neginf=0.0)
                if self.ping_count % 100 == 0 or self.ping_count <= 5:
                    self.logger.warning(f"Filtered {nan_count} NaN/inf values from sidescan data")
            
            # Ensure non-negative values before sqrt
            ping_float = np.clip(ping_float, 0.0, None)
            ping_sqrt = np.sqrt(ping_float)
            
            # Simple per-ping normalization (stable, predictable, no drift)
            # This gives consistent brightness and contrast for each ping
            eps = 1e-6
            ping_min, ping_max = ping_sqrt.min(), ping_sqrt.max()
            
            if ping_max > ping_min + eps:
                # Normal case: normalize to 0-255 range
                ping_norm = ((ping_sqrt - ping_min) / (ping_max - ping_min + eps) * 255.0)
                ping_norm = np.clip(ping_norm, 0.0, 255.0).astype(np.uint8)
            else:
                # Edge case: all values are the same (flat line)
                ping_norm = np.full(ping_sqrt.shape, 128, dtype=np.uint8)
            
            # Apply bronze colormap (ensure 2D input)
            ping_norm_2d = ping_norm.reshape(1, -1)
            ping_colored = self.apply_bronze_colormap(ping_norm_2d)  # Shape: (1, width, 3)
            
            # Initialize buffer on first ping
            if self.waterfall_buffer is None:
                height = self.waterfall_max_height
                width = ping_colored.shape[1]
                self.waterfall_buffer = np.zeros((height, width, 3), dtype=np.uint8)
                self.waterfall_width = width
            
            # Check if width changed - reinitialize buffer if needed
            current_width = ping_colored.shape[1]
            if current_width != self.waterfall_width:
                self.logger.warning(f"Sidescan width changed from {self.waterfall_width} to {current_width}, reinitializing buffer")
                height = self.waterfall_max_height
                self.waterfall_buffer = np.zeros((height, current_width, 3), dtype=np.uint8)
                self.waterfall_width = current_width
                self.waterfall_count = 0
                self.waterfall_filled = False
            
            # Add to circular buffer (newest at index 0)
            # Use slicing instead of np.roll to avoid memory aliasing issues
            if not self.waterfall_filled:
                # Buffer not full yet - only shift the filled portion
                if self.waterfall_count > 0:
                    # Shift down using array slicing (safer than roll)
                    self.waterfall_buffer[1:self.waterfall_count + 1] = self.waterfall_buffer[:self.waterfall_count].copy()
                self.waterfall_buffer[0] = ping_colored[0]
                self.waterfall_count += 1
                if self.waterfall_count >= self.waterfall_max_height:
                    self.waterfall_filled = True
            else:
                # Buffer is full, shift all rows down using slicing
                self.waterfall_buffer[1:] = self.waterfall_buffer[:-1].copy()
                self.waterfall_buffer[0] = ping_colored[0]
            
            # Get the active portion of the buffer for publishing
            # IMPORTANT: Make a copy to prevent view corruption when buffer is modified
            if self.waterfall_filled:
                waterfall_colored = self.waterfall_buffer.copy()
            else:
                # Only show filled portion (first waterfall_count rows)
                waterfall_colored = self.waterfall_buffer[:self.waterfall_count].copy()
            
            height, width, channels = waterfall_colored.shape
            
            # Validate dimensions before publishing
            if height == 0 or width == 0:
                self.logger.warning(f"Invalid waterfall dimensions: {height}x{width}, skipping publish")
                return
            
            # Create Image message (BGR8 encoding for color)
            img_msg = Image()
            img_msg.header = Header()
            img_msg.header.stamp = timestamp.to_msg()
            img_msg.header.frame_id = self.frame_id
            img_msg.height = height
            img_msg.width = width
            img_msg.encoding = 'bgr8'
            img_msg.is_bigendian = 0
            img_msg.step = width * 3  # 3 bytes per pixel (BGR)
            img_msg.data = waterfall_colored.tobytes()
            
            self.intensity_pub.publish(img_msg)
        
        except Exception as e:
            self.logger.error(f"Error publishing sidescan waterfall: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
    
    def apply_bronze_colormap(self, gray_image: np.ndarray) -> np.ndarray:
        """
        Apply bronze colormap to grayscale sonar image.
        
        Maps intensity values to colors:
        - Low intensity (0-64): Dark blue to blue
        - Medium intensity (64-128): Blue to bronze
        - High intensity (128-192): Bronze to yellow
        - Very high (192-255): Yellow to white
        
        Args:
            gray_image: Grayscale image with values 0-255
        
        Returns:
            BGR color image (height x width x 3)
        """
        # Create lookup table for bronze colormap
        lut = np.zeros((256, 3), dtype=np.uint8)
        
        for i in range(256):
            if i < 64:
                # Dark blue to blue (low returns)
                t = i / 64.0
                lut[i] = [int(40 + t * 100), int(t * 30), int(t * 10)]  # BGR
            elif i < 128:
                # Blue to bronze (medium returns)
                t = (i - 64) / 64.0
                lut[i] = [int(140 - t * 60), int(30 + t * 80), int(10 + t * 70)]  # BGR
            elif i < 192:
                # Bronze to yellow (high returns)
                t = (i - 128) / 64.0
                lut[i] = [int(80 - t * 30), int(110 + t * 100), int(80 + t * 130)]  # BGR
            else:
                # Yellow to white (very high returns)
                t = (i - 192) / 63.0
                lut[i] = [int(50 + t * 205), int(210 + t * 45), int(210 + t * 45)]  # BGR
        
        # Apply colormap using lookup table
        colored = lut[gray_image]
        
        return colored
    
    def create_pointcloud2(self, points: np.ndarray, timestamp: Time, frame_id: str = None) -> PointCloud2:
        """
        Create PointCloud2 message from Nx4 array (x, y, z, intensity).
        
        Args:
            points: Nx4 numpy array
            timestamp: ROS timestamp
            frame_id: Frame ID (defaults to self.frame_id)
        
        Returns:
            PointCloud2 message
        """
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = frame_id if frame_id else self.frame_id
        
        msg.height = 1
        msg.width = len(points)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16  # 4 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        msg.data = points.astype(np.float32).tobytes()
        
        return msg
    
    def transform_points_to_odom(self, points_sonar: np.ndarray) -> np.ndarray:
        """
        Transform points from sonar frame to odom frame.
        
        Applies full 6DOF rotation (roll, pitch, yaw) and translation (vehicle position).
        This ensures the sonar swaths stay correctly oriented relative to the vehicle's
        orientation as it moves and rotates in the odom frame.
        
        Args:
            points_sonar: Nx3 array of points in sonar frame (X forward, Y left/right, Z up)
        
        Returns:
            Nx3 array of points in odom frame
        """
        if len(points_sonar) == 0:
            return points_sonar
        
        # Check for NaN in pose data
        if not np.isfinite(self.current_roll) or not np.isfinite(self.current_pitch) or not np.isfinite(self.current_yaw):
            self.get_logger().error(f'Invalid orientation: roll={self.current_roll}, pitch={self.current_pitch}, yaw={self.current_yaw}')
            return np.full_like(points_sonar, np.nan)
        
        if not np.isfinite(self.current_x) or not np.isfinite(self.current_y) or not np.isfinite(self.current_z):
            self.get_logger().error(f'Invalid position: x={self.current_x}, y={self.current_y}, z={self.current_z}')
            return np.full_like(points_sonar, np.nan)
        
        # Create rotation matrices for roll, pitch, yaw (ZYX convention)
        cos_roll = np.cos(self.current_roll)
        sin_roll = np.sin(self.current_roll)
        cos_pitch = np.cos(self.current_pitch)
        sin_pitch = np.sin(self.current_pitch)
        cos_yaw = np.cos(self.current_yaw)
        sin_yaw = np.sin(self.current_yaw)
        
        # Roll (rotation around X axis)
        R_roll = np.array([
            [1, 0,         0        ],
            [0, cos_roll, -sin_roll],
            [0, sin_roll,  cos_roll]
        ])
        
        # Pitch (rotation around Y axis)
        R_pitch = np.array([
            [ cos_pitch, 0, sin_pitch],
            [ 0,         1, 0        ],
            [-sin_pitch, 0, cos_pitch]
        ])
        
        # Yaw (rotation around Z axis)
        R_yaw = np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw,  cos_yaw, 0],
            [0,        0,       1]
        ])
        
        # Combined rotation matrix: R = R_yaw * R_pitch * R_roll
        R = R_yaw @ R_pitch @ R_roll
        
        # Apply rotation to all points
        points_rotated = points_sonar @ R.T
        
        # Apply translation (vehicle position in odom frame)
        vehicle_pos = np.array([
            self.current_x - self.odom_origin_x,
            self.current_y - self.odom_origin_y,
            self.current_z - self.odom_origin_z
        ])
        
        points_odom = points_rotated + vehicle_pos
        
        return points_odom
    
    def update_position_from_data(self, data: DxData):
        """
        Extract position and orientation from sonar ASCII data (NMEA/TSS1).
        
        Parses:
        - $VNYCM: Internal MRU with Yaw, Pitch, Roll
        - :XXAAAASMHHHHQMRRRRSMPPPP: TSS1 with Roll, Pitch
        - $GPGGA/$GPRMC: GPS position
        """
        ascii_data = data.get_ascii_data()
        if not ascii_data:
            return
        
        # Debug: log ASCII data on first few pings
        if self.ping_count <= 3:
            self.logger.info(f"Ping {self.ping_count} ASCII data: {repr(ascii_data[:200])}")
        
        # Parse each sentence
        vnycm_found = False
        heading_found = False
        for sentence in ascii_data.split('\n'):
            sentence = sentence.strip()
            
            # Parse VNYCM (Internal MRU: Yaw, Pitch, Roll)
            if sentence.startswith('$VNYCM'):
                self._parse_vnycm(sentence)
                vnycm_found = True
                heading_found = True
            
            # Parse GPHDT (True Heading) - preferred heading source
            elif sentence.startswith('$GPHDT') or sentence.startswith('$GNHDT'):
                self._parse_gphdt(sentence)
                heading_found = True
            
            # Parse GPRMC (includes heading as "track made good")
            elif sentence.startswith('$GPRMC') or sentence.startswith('$GNRMC'):
                self._parse_gprmc(sentence, set_heading=not heading_found)
            
            # Parse TSS1 (Roll, Pitch)
            elif sentence.startswith(':'):
                self._parse_tss1(sentence)
            
            # Parse GPS position
            elif sentence.startswith('$GPGGA') or sentence.startswith('$GNGGA'):
                self._parse_gpgga(sentence)
        
        # Log heading source periodically
        if self.ping_count % 100 == 0:
            if heading_found:
                self.logger.info(f"Heading: {np.rad2deg(self.current_yaw):.1f}°")
            else:
                self.logger.warning(f"No heading data found in ping {self.ping_count}. Current yaw: {np.rad2deg(self.current_yaw):.1f}°")
        
        # If this is the first ping, set odom origin
        if not self.odom_origin_set:
            self.odom_origin_set = True
            self.odom_origin_x = self.current_x
            self.odom_origin_y = self.current_y
            self.logger.info(f"Odom origin set at ({self.odom_origin_x:.2f}, {self.odom_origin_y:.2f})")
    
    def _parse_vnycm(self, sentence: str):
        """
        Parse $VNYCM sentence: Internal MRU with Yaw, Pitch, Roll.
        
        Format: $VNYCM,+YYY.YYY,+PPP.PPP,+RRR.RRR,...*cc
        
        Updates orientation relative to true north and level horizon.
        Applies meridian convergence to convert true north to grid north (UTM).
        """
        try:
            parts = sentence.split(',')
            if len(parts) >= 4:
                yaw = float(parts[1])  # True heading in degrees
                pitch = float(parts[2])
                roll = float(parts[3])
                
                # Convert degrees to radians and apply meridian convergence
                # Grid yaw = True yaw - convergence
                self.current_yaw = np.deg2rad(yaw) - self.utm_meridian_convergence
                self.current_pitch = np.deg2rad(pitch)
                self.current_roll = np.deg2rad(roll)
                
                # Log periodically to verify heading data is received
                if self.ping_count % 50 == 0:
                    self.logger.info(f"VNYCM: yaw={yaw:.1f}° (true) → {np.rad2deg(self.current_yaw):.1f}° (grid), "
                                   f"pitch={pitch:.1f}° roll={roll:.1f}°")
        except (ValueError, IndexError) as e:
            self.logger.debug(f"Failed to parse VNYCM: {e}")
    
    def _parse_tss1(self, sentence: str):
        """
        Parse TSS1 sentence: Roll and Pitch.
        
        Format: :XXAAAASMHHHHQMRRRRSMPPPP
        - RRRR = Roll in 0.01 degrees
        - PPPP = Pitch in 0.01 degrees
        
        Updates orientation (roll/pitch only, yaw unchanged).
        """
        try:
            if len(sentence) >= 26:
                # Extract roll (positions 15-19, with sign at 14)
                roll_sign = sentence[14]
                roll_str = sentence[15:19]
                roll = float(roll_str) * 0.01  # Convert to degrees
                if roll_sign == '-':
                    roll = -roll
                
                # Extract pitch (positions 21-25, with sign at 20)
                pitch_sign = sentence[20]
                pitch_str = sentence[21:25]
                pitch = float(pitch_str) * 0.01  # Convert to degrees
                if pitch_sign == '-':
                    pitch = -pitch
                
                # Convert to radians and update current orientation
                self.current_roll = np.deg2rad(roll)
                self.current_pitch = np.deg2rad(pitch)
                
                self.logger.debug(f"TSS1: pitch={pitch:.1f}° roll={roll:.1f}°")
        except (ValueError, IndexError) as e:
            self.logger.debug(f"Failed to parse TSS1: {e}")
    
    def _parse_gpgga(self, sentence: str):
        """
        Parse $GPGGA sentence: GPS position.
        
        Format: $GPGGA,hhmmss.ss,ddmm.mmmmm,n,dddmm.mmmmm,l,...*cc
        
        Converts lat/lon to UTM coordinates for proper metric positioning.
        """
        try:
            parts = sentence.split(',')
            if len(parts) >= 6:
                # Latitude
                lat_str = parts[2]
                lat_dir = parts[3]
                if lat_str and lat_dir:
                    lat_deg = float(lat_str[:2])
                    lat_min = float(lat_str[2:])
                    latitude = lat_deg + lat_min / 60.0
                    if lat_dir == 'S':
                        latitude = -latitude
                    
                    # Longitude
                    lon_str = parts[4]
                    lon_dir = parts[5]
                    if lon_str and lon_dir:
                        lon_deg = float(lon_str[:3])
                        lon_min = float(lon_str[3:])
                        longitude = lon_deg + lon_min / 60.0
                        if lon_dir == 'W':
                            longitude = -longitude
                        
                        # Initialize UTM transformer on first GPS fix
                        if self.utm_transformer is None:
                            # Auto-detect UTM zone from first position
                            from pyproj import CRS
                            utm_crs = CRS.from_proj4(f"+proj=utm +zone={int((longitude + 180) / 6) + 1} +datum=WGS84")
                            self.utm_transformer = Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True)
                            self.utm_zone = int((longitude + 180) / 6) + 1
                            
                            # Calculate meridian convergence at this location
                            central_meridian = (self.utm_zone - 1) * 6 - 180 + 3  # Central meridian of UTM zone
                            convergence_deg = np.arctan(np.tan(np.radians(longitude - central_meridian)) * 
                                                       np.sin(np.radians(latitude)))
                            self.utm_meridian_convergence = convergence_deg  # Already in radians
                            
                            self.logger.info(f"Initialized UTM transformer for zone {self.utm_zone}")
                            self.logger.info(f"  Central meridian: {central_meridian:.1f}°")
                            self.logger.info(f"  Meridian convergence: {np.rad2deg(self.utm_meridian_convergence):.3f}° "
                                           f"(grid north is {np.rad2deg(self.utm_meridian_convergence):.3f}° "
                                           f"{'west' if self.utm_meridian_convergence < 0 else 'east'} of true north)")
                        
                        # Convert to UTM (easting, northing in meters)
                        easting, northing = self.utm_transformer.transform(longitude, latitude)
                        self.current_x = easting
                        self.current_y = northing
        except (ValueError, IndexError) as e:
            self.logger.debug(f"Failed to parse GPGGA: {e}")
    
    def _parse_gphdt(self, sentence: str):
        """
        Parse $GPHDT sentence: True Heading.
        
        Format: $GPHDT,x.x,T*cc
        Where x.x is the heading in degrees relative to true north.
        
        NMEA uses NED convention (0° = North, clockwise).
        ROS uses ENU convention (0° = East, counter-clockwise).
        Conversion: yaw_ENU = 90° - heading_NED
        
        Then apply meridian convergence to convert true north to grid north (UTM).
        Grid yaw = True yaw - convergence
        """
        try:
            parts = sentence.split(',')
            if len(parts) >= 2:
                heading_str = parts[1]
                if heading_str:
                    heading_ned = float(heading_str)  # True heading (degrees)
                    # Convert NED heading to ENU yaw
                    heading_enu = 90.0 - heading_ned
                    # Apply meridian convergence correction (convert true north to grid north)
                    heading_grid = heading_enu - np.rad2deg(self.utm_meridian_convergence)
                    self.current_yaw = np.deg2rad(heading_grid)
        except (ValueError, IndexError) as e:
            self.logger.debug(f"Failed to parse GPHDT: {e}")
    
    def _parse_gprmc(self, sentence: str, set_heading: bool = False):
        """
        Parse $GPRMC sentence: Recommended Minimum Navigation Information.
        
        Format: $GPRMC,hhmmss.ss,A,ddmm.mmmm,n,dddmm.mmmm,e,s.s,c.c,ddmmyy,...*cc
        Field 8 (c.c) = Track made good (heading) in degrees relative to true north.
        
        Only sets heading if set_heading=True (when no other heading source available).
        
        NMEA uses NED convention (0° = North, clockwise).
        ROS uses ENU convention (0° = East, counter-clockwise).
        Conversion: yaw_ENU = 90° - heading_NED
        Then apply meridian convergence to convert true north to grid north (UTM).
        """
        try:
            parts = sentence.split(',')
            if len(parts) >= 9 and set_heading:
                heading_str = parts[8]
                if heading_str:
                    heading_ned = float(heading_str)  # True heading
                    # Convert NED heading to ENU yaw
                    heading_enu = 90.0 - heading_ned
                    # Apply meridian convergence correction
                    heading_grid = heading_enu - np.rad2deg(self.utm_meridian_convergence)
                    self.current_yaw = np.deg2rad(heading_grid)
        except (ValueError, IndexError) as e:
            self.logger.debug(f"Failed to parse GPRMC: {e}")
    
    def publish_map_to_odom_tf(self, timestamp: Time):
        """
        Publish TF transform from map frame to odom frame.
        
        The map frame is at the UTM zone origin (0, 0, 0) with standard ENU orientation.
        The odom frame is positioned at the first sonar ping location in UTM coordinates.
        This transform is static (doesn't change after initialization).
        """
        if not self.odom_origin_set:
            return
        
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.map_frame_id
        t.child_frame_id = self.odom_frame_id
        
        # Position: odom origin in UTM coordinates (relative to map at UTM origin)
        t.transform.translation.x = self.odom_origin_x
        t.transform.translation.y = self.odom_origin_y
        t.transform.translation.z = self.odom_origin_z
        
        # Orientation: odom frame has the same orientation as it was initialized
        # (vehicle's orientation at first ping)
        cy = np.cos(self.odom_origin_yaw * 0.5)
        sy = np.sin(self.odom_origin_yaw * 0.5)
        cp = np.cos(self.odom_origin_pitch * 0.5)
        sp = np.sin(self.odom_origin_pitch * 0.5)
        cr = np.cos(self.odom_origin_roll * 0.5)
        sr = np.sin(self.odom_origin_roll * 0.5)
        
        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_tf(self, timestamp: Time):
        """
        Publish TF transform from odom frame to sonar frame.
        
        Maintains the transform tree: map -> odom -> sonar
        Position is relative to the odom origin.
        Orientation is parsed from VNYCM or TSS1 sentences.
        """
        if not self.odom_origin_set:
            # Don't publish TF until odom origin is established
            return
        
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.frame_id
        
        # Position relative to odom origin
        t.transform.translation.x = self.current_x - self.odom_origin_x
        t.transform.translation.y = self.current_y - self.odom_origin_y
        t.transform.translation.z = self.current_z - self.odom_origin_z
        
        # Orientation: Use absolute orientation (from IMU/MRU)
        # This way the sonar frame rotates with the vehicle in world coordinates
        # Using ZYX Euler convention (yaw-pitch-roll)
        cy = np.cos(self.current_yaw * 0.5)
        sy = np.sin(self.current_yaw * 0.5)
        cp = np.cos(self.current_pitch * 0.5)
        sp = np.sin(self.current_pitch * 0.5)
        cr = np.cos(self.current_roll * 0.5)
        sr = np.sin(self.current_roll * 0.5)
        
        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy
        
        self.tf_broadcaster.sendTransform(t)
        
        # Debug: log first few TF publishes
        if self.ping_count <= 3:
            self.logger.info(f"Published TF: pos=({t.transform.translation.x:.2f}, {t.transform.translation.y:.2f}, {t.transform.translation.z:.2f}) "
                           f"quat=({t.transform.rotation.x:.3f}, {t.transform.rotation.y:.3f}, {t.transform.rotation.z:.3f}, {t.transform.rotation.w:.3f})")
    
    def publish_vehicle_position(self, timestamp: Time):
        """
        Publish vehicle position and trajectory path in odom frame.
        
        Publishes both a PoseStamped for current position and a Path
        message with the full trajectory history.
        """
        if not self.odom_origin_set:
            return
        
        msg = PoseStamped()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = self.odom_frame_id
        
        # Position relative to odom origin
        msg.pose.position.x = self.current_x - self.odom_origin_x
        msg.pose.position.y = self.current_y - self.odom_origin_y
        msg.pose.position.z = self.current_z - self.odom_origin_z
        
        # Orientation as quaternion
        cy = np.cos(self.current_yaw * 0.5)
        sy = np.sin(self.current_yaw * 0.5)
        cp = np.cos(self.current_pitch * 0.5)
        sp = np.sin(self.current_pitch * 0.5)
        cr = np.cos(self.current_roll * 0.5)
        sr = np.sin(self.current_roll * 0.5)
        
        msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        # Publish current position
        if self.vehicle_pos_pub.get_subscription_count() > 0:
            self.vehicle_pos_pub.publish(msg)
        
        # Add to trajectory path (downsample to every 5th ping to reduce memory)
        if self.ping_count % 5 == 0:
            self.vehicle_path.poses.append(msg)
            self.vehicle_path.header.stamp = timestamp.to_msg()
            
            # Publish path
            if self.path_pub.get_subscription_count() > 0:
                self.path_pub.publish(self.vehicle_path)
    
    def destroy_node(self):
        """Cleanup on node shutdown."""
        if self.tcp_client:
            self.tcp_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TdssDxDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

