#!/usr/bin/env python3
"""
Record bathymetry point clouds from ROS and save to PLY file.

This node subscribes to the sonar/bathymetry topic and accumulates all points,
then saves them to a PLY file when stopped (Ctrl+C) or on shutdown.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os
from datetime import datetime
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs


class PointCloudRecorder(Node):
    """Records point clouds and saves to PLY file."""
    
    def __init__(self):
        super().__init__('pointcloud_recorder')
        
        # Declare parameters
        self.declare_parameter('output_dir', os.path.expanduser('~/sonar_data'))
        self.declare_parameter('output_filename', '')  # Auto-generate if empty
        self.declare_parameter('frame_id', 'map')  # Frame to save points in
        self.declare_parameter('input_topic', 'sonar/bathymetry_filtered')  # Subscribe to filtered by default
        
        # Get parameters
        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        self.output_filename = self.get_parameter('output_filename').value
        self.target_frame = self.get_parameter('frame_id').value
        self.input_topic = self.get_parameter('input_topic').value
        
        # Create output directory if needed
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Storage for accumulated points
        self.points = []  # List of (x, y, z, intensity) tuples
        self.point_count = 0
        self.ping_count = 0
        
        # TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS profile matching the driver (BEST_EFFORT)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to bathymetry topic
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            sensor_qos
        )
        
        self.get_logger().info(f'Recording point clouds from: {self.input_topic}')
        self.get_logger().info(f'Saving to: {self.output_dir}')
        self.get_logger().info(f'Target frame: {self.target_frame}')
        self.get_logger().info('Press Ctrl+C to save and exit')
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Accumulate points from incoming point cloud."""
        try:
            # Get transform from source frame to target frame
            transform = None
            source_frame = msg.header.frame_id
            
            if source_frame != self.target_frame:
                try:
                    # Look up transform at the time of the point cloud
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        source_frame,
                        msg.header.stamp,
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                except Exception as e:
                    if self.ping_count == 0:
                        self.get_logger().warning(f'Could not transform from {source_frame} to {self.target_frame}: {e}')
                        self.get_logger().warning('Recording points in source frame')
                    transform = None
            
            # Debug: Log field names on first message
            if self.ping_count == 0:
                field_names = [field.name for field in msg.fields]
                self.get_logger().info(f'PointCloud2 fields: {field_names}')
                self.get_logger().info(f'Source frame: {source_frame}, Target frame: {self.target_frame}')
                self.get_logger().info(f'Transform available: {transform is not None}')
            
            # Extract points - don't skip nans, we'll filter manually
            points_gen = pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=False)
            
            for point in points_gen:
                x, y, z, intensity = point
                
                # Filter NaN/inf in ALL fields including intensity
                if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z) and np.isfinite(intensity)):
                    continue
                
                # Apply transform if available
                if transform:
                    # Extract rotation (quaternion) and translation
                    t = transform.transform.translation
                    q = transform.transform.rotation
                    
                    # Convert quaternion to rotation matrix
                    # q = (x, y, z, w)
                    qx, qy, qz, qw = q.x, q.y, q.z, q.w
                    
                    # Rotation matrix from quaternion
                    R = np.array([
                        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
                        [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
                        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
                    ])
                    
                    # Apply rotation then translation
                    point_vec = np.array([x, y, z])
                    rotated = R @ point_vec
                    x_transformed = rotated[0] + t.x
                    y_transformed = rotated[1] + t.y
                    z_transformed = rotated[2] + t.z
                else:
                    x_transformed = x
                    y_transformed = y
                    z_transformed = z
                
                # Debug first valid point
                if self.ping_count == 0 and self.point_count == 0:
                    self.get_logger().info(f'First point (source): x={x:.3f}, y={y:.3f}, z={z:.3f}')
                    if transform:
                        self.get_logger().info(f'First point (target): x={x_transformed:.3f}, y={y_transformed:.3f}, z={z_transformed:.3f}')
                
                # Filter outliers - reasonable range for underwater sonar (±2000m horizontal, ±100m vertical)
                if abs(x_transformed) > 2000.0 or abs(y_transformed) > 2000.0 or abs(z_transformed) > 100.0:
                    continue
                
                # Store as tuple (in target frame)
                self.points.append((float(x_transformed), float(y_transformed), float(z_transformed), float(intensity)))
                self.point_count += 1
            
            self.ping_count += 1
            
            # Log progress every 100 pings
            if self.ping_count % 100 == 0:
                self.get_logger().info(f'Recorded {self.ping_count} pings, {self.point_count} valid points')
        
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def save_ply(self):
        """Save accumulated points to multiple file formats."""
        if len(self.points) == 0:
            self.get_logger().warning('No points to save!')
            return
        
        # Generate filename if not specified
        if not self.output_filename:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            base_filename = f'sonar_bathymetry_{timestamp}'
        else:
            base_filename = self.output_filename.rsplit('.', 1)[0]  # Remove extension if present
        
        try:
            # Convert to numpy array directly - use float64 to avoid overflow
            raw_array = np.array(self.points, dtype=np.float64)  # Shape: (N, 4)
            
            self.get_logger().info(f'Point cloud statistics:')
            self.get_logger().info(f'  Total points: {len(raw_array)}')
            self.get_logger().info(f'  X range: [{raw_array[:, 0].min():.3f}, {raw_array[:, 0].max():.3f}] m')
            self.get_logger().info(f'  Y range: [{raw_array[:, 1].min():.3f}, {raw_array[:, 1].max():.3f}] m')
            self.get_logger().info(f'  Z range: [{raw_array[:, 2].min():.3f}, {raw_array[:, 2].max():.3f}] m')
            self.get_logger().info(f'  Centroid: ({raw_array[:, 0].mean():.3f}, {raw_array[:, 1].mean():.3f}, {raw_array[:, 2].mean():.3f})')
            
            # Convert to float32 for saving (standard for point clouds)
            raw_array = raw_array.astype(np.float32)
            
            # Save PLY (ASCII)
            ply_path = os.path.join(self.output_dir, f'{base_filename}.ply')
            self._save_ply_ascii(ply_path, raw_array)
            self.get_logger().info(f'✓ Saved PLY: {ply_path}')
            
            # Save XYZ (simple text format)
            xyz_path = os.path.join(self.output_dir, f'{base_filename}.xyz')
            self._save_xyz(xyz_path, raw_array)
            self.get_logger().info(f'✓ Saved XYZ: {xyz_path}')
            
            # Save PCD (Point Cloud Data format)
            pcd_path = os.path.join(self.output_dir, f'{base_filename}.pcd')
            self._save_pcd(pcd_path, raw_array)
            self.get_logger().info(f'✓ Saved PCD: {pcd_path}')
            
            self.get_logger().info(f'\nOpen with:')
            self.get_logger().info(f'  cloudcompare {ply_path}')
            self.get_logger().info(f'  meshlab {ply_path}')
            self.get_logger().info(f'  pcl_viewer {pcd_path}')
        
        except Exception as e:
            self.get_logger().error(f'Error saving point cloud: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _save_ply_ascii(self, filepath, points_array):
        """Save as PLY ASCII format. points_array shape: (N, 4) [x, y, z, intensity]"""
        with open(filepath, 'w') as f:
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'element vertex {len(points_array)}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('property float intensity\n')
            f.write('end_header\n')
            
            for point in points_array:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.6f}\n")
    
    def _save_xyz(self, filepath, points_array):
        """Save as simple XYZ text format (x y z intensity per line). points_array shape: (N, 4)"""
        with open(filepath, 'w') as f:
            for point in points_array:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.6f}\n")
    
    def _save_pcd(self, filepath, points_array):
        """Save as PCD (Point Cloud Data) format. points_array shape: (N, 4)"""
        with open(filepath, 'w') as f:
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z intensity\n')
            f.write('SIZE 4 4 4 4\n')
            f.write('TYPE F F F F\n')
            f.write('COUNT 1 1 1 1\n')
            f.write(f'WIDTH {len(points_array)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(points_array)}\n')
            f.write('DATA ascii\n')
            
            for point in points_array:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.6f}\n")
    
    def destroy_node(self):
        """Save points before shutting down."""
        self.get_logger().info('Shutting down, saving point cloud...')
        self.save_ply()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    recorder = PointCloudRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
