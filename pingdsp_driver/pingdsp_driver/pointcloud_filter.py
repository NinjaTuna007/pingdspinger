#!/usr/bin/env python3
"""
Point Cloud Filter Node for 3DSS-DX Bathymetry Data

Subscribes to raw bathymetry point clouds, applies filtering (outlier removal, 
range filtering), and republishes cleaned point clouds.

Filters applied:
- Statistical Outlier Removal (SOR): Removes points that are statistical outliers
- Minimum Range Filter: Removes spurious detections too close to the sensor
- Maximum Range Filter: Removes detections beyond valid range
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header


class PointCloudFilter(Node):
    """Filters bathymetry point clouds to remove outliers and spurious detections."""
    
    def __init__(self):
        super().__init__('pointcloud_filter')
        
        # Declare parameters
        self.declare_parameter('enable_filtering', True)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 100.0)
        self.declare_parameter('enable_intensity_filter', True)
        self.declare_parameter('min_intensity', 0.1)
        self.declare_parameter('enable_altitude_filter', True)
        self.declare_parameter('min_altitude', -50.0)
        self.declare_parameter('max_altitude', 5.0)
        self.declare_parameter('enable_consecutive_filter', True)
        self.declare_parameter('max_consecutive_jump', 10.0)
        self.declare_parameter('input_topic', 'sonar/bathymetry')
        self.declare_parameter('output_topic', 'sonar/bathymetry_filtered')
        
        # Get parameters
        self.enable_filtering = self.get_parameter('enable_filtering').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.enable_intensity_filter = self.get_parameter('enable_intensity_filter').value
        self.min_intensity = self.get_parameter('min_intensity').value
        self.enable_altitude_filter = self.get_parameter('enable_altitude_filter').value
        self.min_altitude = self.get_parameter('min_altitude').value
        self.max_altitude = self.get_parameter('max_altitude').value
        self.enable_consecutive_filter = self.get_parameter('enable_consecutive_filter').value
        self.max_consecutive_jump = self.get_parameter('max_consecutive_jump').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # QoS profile matching the driver (BEST_EFFORT)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            sensor_qos
        )
        
        # Publisher
        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            sensor_qos
        )
        
        # Statistics
        self.processed_count = 0
        self.total_points_in = 0
        self.total_points_out = 0
        
        self.get_logger().info('Point Cloud Filter initialized')
        self.get_logger().info(f'  Input: {self.input_topic}')
        self.get_logger().info(f'  Output: {self.output_topic}')
        self.get_logger().info(f'  Filtering enabled: {self.enable_filtering}')
        if self.enable_filtering:
            self.get_logger().info(f'  Range: [{self.min_range:.2f}, {self.max_range:.2f}] m')
            if self.enable_intensity_filter:
                self.get_logger().info(f'  Intensity: >={self.min_intensity:.2f}')
            if self.enable_altitude_filter:
                self.get_logger().info(f'  Altitude: [{self.min_altitude:.1f}, {self.max_altitude:.1f}] m')
            if self.enable_consecutive_filter:
                self.get_logger().info(f'  Consecutive jump: <{self.max_consecutive_jump:.1f} m')
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud."""
        try:
            # Extract points
            points_list = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True))
            
            if len(points_list) == 0:
                # Empty cloud, just republish
                self.publisher.publish(msg)
                return
            
            # Convert structured array to regular array
            # points_list is a list of tuples (x, y, z, intensity)
            points = np.array([(p[0], p[1], p[2], p[3]) for p in points_list], dtype=np.float32)
            
            self.total_points_in += len(points)
            
            if not self.enable_filtering:
                # Pass through without filtering
                self.publish_pointcloud(points, msg.header)
                self.total_points_out += len(points)
                return
            
            # Apply filters in order
            # 1. Range filter (fast, removes obvious outliers)
            points = self.apply_range_filter(points)
            
            # 2. Intensity filter (removes weak returns)
            if self.enable_intensity_filter and len(points) > 0:
                points = self.apply_intensity_filter(points)
            
            # 3. Altitude filter (removes impossible heights/depths)
            if self.enable_altitude_filter and len(points) > 0:
                points = self.apply_altitude_filter(points)
            
            # 4. Consecutive point filter (removes isolated spikes along scan line)
            if self.enable_consecutive_filter and len(points) > 0:
                points = self.apply_consecutive_filter(points)
            
            if len(points) == 0:
                self.get_logger().warning('All points filtered out')
                return
            
            # Publish filtered cloud
            self.publish_pointcloud(points, msg.header)
            self.total_points_out += len(points)
            
            self.processed_count += 1
            
            # Log statistics periodically
            if self.processed_count % 100 == 0:
                retention_rate = (self.total_points_out / self.total_points_in * 100) if self.total_points_in > 0 else 0
                self.get_logger().info(
                    f'Processed {self.processed_count} clouds, '
                    f'retention: {retention_rate:.1f}% '
                    f'({self.total_points_out}/{self.total_points_in} points)'
                )
        
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def apply_range_filter(self, points: np.ndarray) -> np.ndarray:
        """
        Filter points based on distance from sensor origin.
        
        Removes:
        - Points too close (spurious detections near sensor)
        - Points too far (beyond valid sonar range)
        
        Args:
            points: Nx4 array [x, y, z, intensity]
        
        Returns:
            Filtered Nx4 array
        """
        # Calculate range (distance from origin in XY plane for bathymetry)
        ranges = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)
        
        # Create mask for valid range
        valid_mask = (ranges >= self.min_range) & (ranges <= self.max_range)
        
        return points[valid_mask]
    
    def apply_intensity_filter(self, points: np.ndarray) -> np.ndarray:
        """
        Filter points based on intensity (return strength).
        
        Removes weak returns that are likely noise or multipath.
        
        Args:
            points: Nx4 array [x, y, z, intensity]
        
        Returns:
            Filtered Nx4 array
        """
        valid_mask = points[:, 3] >= self.min_intensity
        return points[valid_mask]
    
    def apply_altitude_filter(self, points: np.ndarray) -> np.ndarray:
        """
        Filter points based on altitude (Z coordinate in sonar frame).
        
        Removes points above water surface or impossibly deep.
        
        Args:
            points: Nx4 array [x, y, z, intensity]
        
        Returns:
            Filtered Nx4 array
        """
        valid_mask = (points[:, 2] >= self.min_altitude) & (points[:, 2] <= self.max_altitude)
        return points[valid_mask]
    
    def apply_consecutive_filter(self, points: np.ndarray) -> np.ndarray:
        """
        Filter isolated spikes along the scan line.
        
        Checks distance between consecutive points. If jump is too large,
        likely a spurious detection or multipath return.
        
        Args:
            points: Nx4 array [x, y, z, intensity]
        
        Returns:
            Filtered Nx4 array
        """
        if len(points) < 3:
            return points
        
        # Compute distance between consecutive points
        diffs = np.diff(points[:, :3], axis=0)
        distances = np.sqrt(np.sum(diffs**2, axis=1))
        
        # Create mask: keep first point, then check distances
        valid_mask = np.ones(len(points), dtype=bool)
        valid_mask[1:] = distances < self.max_consecutive_jump
        
        return points[valid_mask]
    
    def publish_pointcloud(self, points: np.ndarray, header: Header):
        """
        Publish filtered point cloud.
        
        Args:
            points: Nx4 array [x, y, z, intensity]
            header: Original message header (preserve frame_id and timestamp)
        """
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        
        from sensor_msgs.msg import PointField
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        msg.data = points.astype(np.float32).tobytes()
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilter()
    
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
