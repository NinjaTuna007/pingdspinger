#!/usr/bin/env python3
"""
Stream bathymetry point clouds to a UTM-referenced .xyz file for CloudCompare.

This node subscribes to the (filtered) bathymetry point cloud, transforms each
ping from the sonar frame into a global, UTM-aligned frame (``utm`` for the SBG
stack, falling back to ``map`` for the sonar-only setup), and appends the points
to a single ``.xyz`` file as they arrive. Because it writes incrementally and
flushes, killing the session at any time (Ctrl+C, ``tmux kill-session``, etc.)
leaves a complete, openable file - there is no separate "save on shutdown" step
that can be skipped by SIGTERM.

Output columns are ``easting northing z intensity`` in metres (UTM), float64
precision. When you load the file in CloudCompare it will offer to apply a
"global shift" because the coordinates are large - accept it; the data stays
georeferenced and lossless.

Noise gating: spurious returns are rejected using bounds on the *sonar-head
relative* coordinates (metres from the transducer / depth), i.e. before the UTM
transform, so the gate keeps working regardless of where on the globe we are.
"""

import os
from datetime import datetime

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2

import sensor_msgs_py.point_cloud2 as pc2

from tf2_ros import Buffer, TransformListener


class PointCloudRecorder(Node):
    """Stream point clouds to a UTM-referenced .xyz file."""

    def __init__(self):
        """Open the output stream and subscribe to the bathymetry cloud."""
        super().__init__('pointcloud_recorder')

        # Output
        self.declare_parameter(
            'output_dir', os.path.expanduser('~/sonar_data'))
        self.declare_parameter('output_filename', '')  # auto-generate if empty
        self.declare_parameter('input_topic', 'sonar/bathymetry_filtered')

        # Global UTM-aligned frame to record in. The first candidate that TF can
        # resolve from the cloud's source frame is used: 'utm' for the SBG stack,
        # 'map' for the sonar-only stack (which is also UTM-valued).
        self.declare_parameter('target_frame', 'utm')
        self.declare_parameter('fallback_frames', ['map'])

        # Noise gate on sonar-head-relative coordinates (pre-transform metres).
        self.declare_parameter('max_horizontal', 2000.0)  # |x|,|y| from head
        self.declare_parameter('max_abs_z', 100.0)        # |depth| from head

        # Flush cadence (pings). 1 == flush every ping (safest).
        self.declare_parameter('flush_every', 1)

        self.output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value)
        self.output_filename = self.get_parameter('output_filename').value
        self.input_topic = self.get_parameter('input_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.fallback_frames = list(
            self.get_parameter('fallback_frames').value)
        self.max_horizontal = float(
            self.get_parameter('max_horizontal').value)
        self.max_abs_z = float(self.get_parameter('max_abs_z').value)
        self.flush_every = max(1, int(self.get_parameter('flush_every').value))

        os.makedirs(self.output_dir, exist_ok=True)

        # Resolved once we see the first cloud and a usable TF.
        self.resolved_frame = None
        self.point_count = 0
        self.ping_count = 0
        # Running bounds/centroid for the shutdown summary.
        self._min = np.array([np.inf, np.inf, np.inf])
        self._max = np.array([-np.inf, -np.inf, -np.inf])
        self._sum = np.zeros(3)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Open the output file up front and stream into it.
        if self.output_filename:
            base = self.output_filename.rsplit('.', 1)[0]
        else:
            base = 'sonar_bathymetry_' + \
                datetime.now().strftime('%Y%m%d_%H%M%S')
        self.xyz_path = os.path.join(self.output_dir, base + '.xyz')
        self.xyz_file = open(self.xyz_path, 'w')
        self.xyz_file.write(
            '# PingDSP bathymetry point cloud\n'
            '# frame: UTM (easting/northing in metres); '
            'load in CloudCompare and accept the global shift\n'
            '# columns: easting northing z intensity\n')
        self.xyz_file.flush()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self.pointcloud_callback,
            sensor_qos)

        self.get_logger().info(f'Recording point clouds from: {self.input_topic}')
        self.get_logger().info(f'Streaming UTM .xyz to: {self.xyz_path}')
        self.get_logger().info(
            f'Target frame: {self.target_frame} '
            f'(fallbacks: {self.fallback_frames})')
        self.get_logger().info(
            f'Sonar-relative noise gate: |x|,|y| <= {self.max_horizontal} m, '
            f'|z| <= {self.max_abs_z} m')
        self.get_logger().info('Kill the session (Ctrl+C) to finalize.')

    def _resolve_frame(self, source_frame, stamp):
        """Pick the first global frame TF can resolve from ``source_frame``."""
        candidates = [self.target_frame] + self.fallback_frames
        for frame in candidates:
            if not frame:
                continue
            try:
                self.tf_buffer.lookup_transform(
                    frame, source_frame, stamp,
                    timeout=Duration(seconds=0.1))
                return frame
            except Exception:
                try:
                    self.tf_buffer.lookup_transform(
                        frame, source_frame, Time())
                    return frame
                except Exception:
                    continue
        return None

    def _lookup(self, source_frame, stamp):
        """Transform source -> resolved frame; stamped first, else latest."""
        try:
            return self.tf_buffer.lookup_transform(
                self.resolved_frame, source_frame, stamp,
                timeout=Duration(seconds=0.1))
        except Exception:
            return self.tf_buffer.lookup_transform(
                self.resolved_frame, source_frame, Time())

    @staticmethod
    def _matrix(transform):
        """Build (R, t) from a TransformStamped."""
        q = transform.transform.rotation
        t = transform.transform.translation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz),
             2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz),
             2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx),
             1 - 2 * (qx * qx + qy * qy)],
        ], dtype=np.float64)
        return R, np.array([t.x, t.y, t.z], dtype=np.float64)

    def pointcloud_callback(self, msg: PointCloud2):
        """Gate in the sonar frame, transform survivors to UTM, append."""
        try:
            source_frame = msg.header.frame_id

            if self.resolved_frame is None:
                self.resolved_frame = self._resolve_frame(
                    source_frame, msg.header.stamp)
                if self.resolved_frame is None:
                    if self.ping_count == 0:
                        self.get_logger().warning(
                            'No TF from '
                            f'{source_frame} to {self.target_frame} or '
                            f'{self.fallback_frames} yet; waiting...')
                    return
                self.get_logger().info(
                    f'Recording in frame: {self.resolved_frame}')

            try:
                transform = self._lookup(source_frame, msg.header.stamp)
            except Exception as e:  # noqa: BLE001
                if self.ping_count == 0:
                    self.get_logger().warning(
                        f'TF {source_frame} -> {self.resolved_frame} '
                        f'unavailable: {e}')
                return

            # Read the whole cloud as float64 Nx4.
            pts = pc2.read_points(
                msg, field_names=('x', 'y', 'z', 'intensity'),
                skip_nans=False)
            arr = np.array(
                [(p[0], p[1], p[2], p[3]) for p in pts], dtype=np.float64)
            if arr.size == 0:
                self.ping_count += 1
                return

            xyz = arr[:, :3]
            intensity = arr[:, 3]

            # Noise gate on sonar-head-relative coords (pre-transform).
            mask = np.isfinite(arr).all(axis=1)
            mask &= np.abs(xyz[:, 0]) <= self.max_horizontal
            mask &= np.abs(xyz[:, 1]) <= self.max_horizontal
            mask &= np.abs(xyz[:, 2]) <= self.max_abs_z
            if not mask.any():
                self.ping_count += 1
                return

            xyz = xyz[mask]
            intensity = intensity[mask]

            # Transform survivors to the global UTM-aligned frame.
            R, t = self._matrix(transform)
            world = xyz @ R.T + t  # (N,3) float64

            # Append to the .xyz stream.
            out = np.column_stack((world, intensity))
            np.savetxt(self.xyz_file, out, fmt='%.3f')

            n = world.shape[0]
            self.point_count += n
            self._min = np.minimum(self._min, world.min(axis=0))
            self._max = np.maximum(self._max, world.max(axis=0))
            self._sum += world.sum(axis=0)

            self.ping_count += 1
            if self.ping_count % self.flush_every == 0:
                self.xyz_file.flush()

            if self.ping_count % 100 == 0:
                self.get_logger().info(
                    f'Recorded {self.ping_count} pings, '
                    f'{self.point_count} points')

        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Error processing point cloud: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _finalize(self):
        """Flush, close, and log a summary of the streamed file."""
        if self.xyz_file is None:
            return
        try:
            self.xyz_file.flush()
            self.xyz_file.close()
        except Exception:
            pass
        self.xyz_file = None

        if self.point_count == 0:
            self.get_logger().warning(
                f'No points recorded; {self.xyz_path} is empty.')
            return

        centroid = self._sum / self.point_count
        self.get_logger().info('Point cloud (UTM) statistics:')
        self.get_logger().info(f'  Total points: {self.point_count}')
        self.get_logger().info(
            f'  Easting  range: [{self._min[0]:.3f}, {self._max[0]:.3f}] m')
        self.get_logger().info(
            f'  Northing range: [{self._min[1]:.3f}, {self._max[1]:.3f}] m')
        self.get_logger().info(
            f'  Z        range: [{self._min[2]:.3f}, {self._max[2]:.3f}] m')
        self.get_logger().info(
            f'  Centroid: ({centroid[0]:.3f}, {centroid[1]:.3f}, '
            f'{centroid[2]:.3f})')
        self.get_logger().info(f'Saved UTM XYZ: {self.xyz_path}')
        self.get_logger().info(
            f'  Open with: cloudcompare {self.xyz_path} '
            '(accept the global shift prompt)')

    def destroy_node(self):
        """Finalize the stream before shutting down."""
        self.get_logger().info('Shutting down, finalizing point cloud...')
        self._finalize()
        super().destroy_node()


def main(args=None):
    """Spin the recorder, finalizing the stream on exit."""
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
