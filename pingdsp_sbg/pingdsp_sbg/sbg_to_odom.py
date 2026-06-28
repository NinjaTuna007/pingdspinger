#!/usr/bin/env python3
"""Convert the SBG (NED) EKF stream into a ROS ENU odometry + TF tree.

Python re-implementation of evolo_common's ``sbg_to_odom`` (originally
C++/Eigen) using ``transforms3d`` for all rotation math (see
:mod:`pingdsp_sbg.sbg_transforms`).

Pipeline:

* look up the static ``utm -> <prefix>/odom`` transform published by
  ``sbg_to_odom_initializer`` once, to recover the UTM datum offset;
* per ``SbgEkfNav`` fix: UTM-forward the lat/lon, subtract the datum offset to
  get a local ENU position;
* attitude: from ``SbgEkfQuat`` when present, else ``SbgEkfEuler`` (the
  PingDSP Ellipse streams Euler, not quaternion) -- both converted NED->ENU;
* per ``SbgImuData`` / ``SbgImuShort``: convert body angular rates to FLU;
* on a timer publish ``nav_msgs/Odometry`` (``<prefix>/odom`` ->
  ``<prefix>/base_link``), the matching dynamic TF, and auxiliary
  heading/course/speed/latlon topics.

Unlike the evolo original we also populate pose/twist covariance from the SBG
1-sigma accuracy fields.
"""

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from pingdsp_sbg import sbg_transforms as st
import rclpy
from rclpy.node import Node
from sbg_driver.msg import (SbgEkfEuler, SbgEkfNav, SbgEkfQuat, SbgImuData,
                            SbgImuShort)
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class SbgToOdom(Node):
    """Publishes ENU odometry + TF from the SBG NED EKF stream."""

    def __init__(self):
        super().__init__('sbg_to_odom')

        self.declare_parameter('frame_prefix', 'pingdsp')
        self.declare_parameter('ekf_nav_topic', 'sbg/ekf_nav')
        self.declare_parameter('ekf_quat_topic', 'sbg/ekf_quat')
        self.declare_parameter('ekf_euler_topic', 'sbg/ekf_euler')
        self.declare_parameter('imu_topic', 'sbg/imu_data')
        self.declare_parameter('imu_short_topic', 'sbg/imu_short')
        # Where orientation comes from: 'quat' (SbgEkfQuat only), 'euler'
        # (SbgEkfEuler only), or 'auto' (use quat when present, else euler).
        # The PingDSP Ellipse streams EKF_EULER (+ IMU_SHORT) but not EKF_QUAT,
        # so 'auto' makes the stack work as-recorded without device reconfig.
        self.declare_parameter('attitude_source', 'auto')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_covariance', True)

        prefix = self.get_parameter('frame_prefix').value
        nav_topic = self.get_parameter('ekf_nav_topic').value
        quat_topic = self.get_parameter('ekf_quat_topic').value
        euler_topic = self.get_parameter('ekf_euler_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        imu_short_topic = self.get_parameter('imu_short_topic').value
        self.attitude_source = str(
            self.get_parameter('attitude_source').value).lower()
        odom_topic = self.get_parameter('odom_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.publish_cov = bool(self.get_parameter('publish_covariance').value)

        self.odom_frame = '{}/odom'.format(prefix)
        self.base_frame = '{}/base_link'.format(prefix)

        # Datum offset (UTM easting/northing of <prefix>/odom origin).
        self.utm_ready = False
        self.x_offset = 0.0
        self.y_offset = 0.0

        # Latest state.
        self.have_position = False
        self.have_orientation = False
        # When attitude_source == 'auto' and a quaternion has arrived recently,
        # ignore Euler updates so the (redundant) quat wins. Reset every quat.
        self._last_quat_time = None
        self._quat_priority_sec = 0.5
        self.x = 0.0
        self.y = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.q_sbg_xyzw = (0.0, 0.0, 0.0, 1.0)
        self.q_enu_xyzw = (0.0, 0.0, 0.0, 1.0)
        self.heading_deg = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.gyro = (0.0, 0.0, 0.0)
        self.pos_acc = (0.0, 0.0, 0.0)
        self.vel_acc = (0.0, 0.0, 0.0)
        self.att_acc = (0.0, 0.0, 0.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.heading_pub = self.create_publisher(Float32, 'heading', 10)
        self.course_pub = self.create_publisher(Float32, 'course', 10)
        self.speed_pub = self.create_publisher(Float32, 'speed', 10)
        self.latlon_pub = self.create_publisher(GeoPoint, 'latlon', 10)

        self.create_subscription(SbgEkfNav, nav_topic, self.nav_callback, 10)
        if self.attitude_source in ('auto', 'quat'):
            self.create_subscription(
                SbgEkfQuat, quat_topic, self.quat_callback, 10)
        if self.attitude_source in ('auto', 'euler'):
            self.create_subscription(
                SbgEkfEuler, euler_topic, self.euler_callback, 10)
        # Gyro can come from the legacy IMU_DATA log or the modern IMU_SHORT
        # log; subscribe to both and use whichever the device actually streams.
        self.create_subscription(SbgImuData, imu_topic, self.imu_callback, 10)
        self.create_subscription(
            SbgImuShort, imu_short_topic, self.imu_short_callback, 10)

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.1
        self.timer = self.create_timer(period, self.publish)

        self.get_logger().info(
            'sbg_to_odom: {} -> {} (attitude_source={}, waiting for '
            'utm -> {} datum TF)'.format(
                self.odom_frame, self.base_frame, self.attitude_source,
                self.odom_frame))

    def _ensure_utm_offset(self):
        """Look up the static ``utm -> <prefix>/odom`` datum offset once."""
        if self.utm_ready:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(
                'utm', self.odom_frame, rclpy.time.Time())
        except Exception:
            return False
        self.x_offset = tf.transform.translation.x
        self.y_offset = tf.transform.translation.y
        self.utm_ready = True
        self.get_logger().info(
            'Datum offset acquired: easting={:.2f} northing={:.2f}'.format(
                self.x_offset, self.y_offset))
        return True

    def nav_callback(self, msg: SbgEkfNav):
        """Convert a fix to a local ENU position + body velocity."""
        if not self._ensure_utm_offset():
            return
        easting, northing, _, _ = st.latlon_to_utm(
            msg.latitude, msg.longitude)
        self.x = easting - self.x_offset
        self.y = northing - self.y_offset
        self.latitude = msg.latitude
        self.longitude = msg.longitude

        vel_ned = (msg.velocity.x, msg.velocity.y, msg.velocity.z)
        self.vx, self.vy, self.vz = st.sbg_velocity_to_body_enu(
            self.q_sbg_xyzw, vel_ned)

        self.pos_acc = (msg.position_accuracy.x,
                        msg.position_accuracy.y,
                        msg.position_accuracy.z)
        self.vel_acc = (msg.velocity_accuracy.x,
                        msg.velocity_accuracy.y,
                        msg.velocity_accuracy.z)
        self.have_position = True

    def _set_attitude(self, q_sbg_xyzw, att_acc):
        """Store an attitude (SBG NED quat) and derive the ENU pose/heading."""
        self.q_sbg_xyzw = q_sbg_xyzw
        self.q_enu_xyzw = st.sbg_quat_to_enu_quat(q_sbg_xyzw)
        self.heading_deg = st.enu_quat_to_heading_deg(self.q_enu_xyzw)
        self.att_acc = att_acc
        self.have_orientation = True

    def quat_callback(self, msg: SbgEkfQuat):
        """Convert the NED attitude quaternion to ENU (preferred source)."""
        self._last_quat_time = self.get_clock().now()
        self._set_attitude(
            (msg.quaternion.x, msg.quaternion.y,
             msg.quaternion.z, msg.quaternion.w),
            (msg.accuracy.x, msg.accuracy.y, msg.accuracy.z))

    def euler_callback(self, msg: SbgEkfEuler):
        """Convert NED Euler attitude to ENU (used when no EKF_QUAT streams)."""
        # In 'auto' mode let a live quaternion stream take precedence.
        if self.attitude_source == 'auto' and self._last_quat_time is not None:
            dt = (self.get_clock().now() - self._last_quat_time).nanoseconds
            if dt < self._quat_priority_sec * 1e9:
                return
        q_sbg = st.sbg_euler_to_quat_xyzw(
            msg.angle.x, msg.angle.y, msg.angle.z)
        self._set_attitude(
            q_sbg, (msg.accuracy.x, msg.accuracy.y, msg.accuracy.z))

    def imu_callback(self, msg: SbgImuData):
        """Convert body angular rates (FRD) to FLU (legacy IMU_DATA log)."""
        self.gyro = st.sbg_gyro_to_enu(
            (msg.gyro.x, msg.gyro.y, msg.gyro.z))

    def imu_short_callback(self, msg: SbgImuShort):
        """Convert body angular rates from the IMU_SHORT log (FRD) to FLU.

        The sbg_driver already scales ``delta_angle`` to rad/s and leaves it in
        the NED/FRD body frame (use_enu false), matching SbgImuData.gyro.
        """
        self.gyro = st.sbg_gyro_to_enu(
            (msg.delta_angle.x, msg.delta_angle.y, msg.delta_angle.z))

    def _fill_covariance(self, odom):
        """Populate pose/twist covariance diagonals from SBG 1-sigma accuracy.

        Position accuracy is reported in NED (x=North, y=East); after the ENU
        swap the local-x variance comes from East and local-y from North.
        """
        ax, ay, az = self.pos_acc
        # ENU x <- East (NED y), ENU y <- North (NED x).
        odom.pose.covariance[0] = ay * ay
        odom.pose.covariance[7] = ax * ax
        odom.pose.covariance[14] = az * az
        rr, pp, yy = self.att_acc
        odom.pose.covariance[21] = rr * rr
        odom.pose.covariance[28] = pp * pp
        odom.pose.covariance[35] = yy * yy

        vax, vay, vaz = self.vel_acc
        odom.twist.covariance[0] = vay * vay
        odom.twist.covariance[7] = vax * vax
        odom.twist.covariance[14] = vaz * vaz

    def publish(self):
        """Publish odometry, dynamic TF, and auxiliary nav topics."""
        if not (self.utm_ready and self.have_position
                and self.have_orientation):
            return

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = self.q_enu_xyzw

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = self.vz
        odom.twist.twist.angular.x = self.gyro[0]
        odom.twist.twist.angular.y = self.gyro[1]
        odom.twist.twist.angular.z = self.gyro[2]
        if self.publish_cov:
            self._fill_covariance(odom)
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        self.heading_pub.publish(Float32(data=float(self.heading_deg)))
        self.course_pub.publish(Float32(data=float(
            st.course_deg(self.heading_deg, self.vx, self.vy))))
        self.speed_pub.publish(Float32(data=float(
            st.speed_mps(self.vx, self.vy))))
        self.latlon_pub.publish(GeoPoint(
            latitude=float(self.latitude),
            longitude=float(self.longitude),
            altitude=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = SbgToOdom()
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
