#!/usr/bin/env python3
"""Anchor a local metric frame to the first valid SBG GNSS fix.

Mirrors evolo_common's ``sbg_to_odom_initializer``: it waits for the first
``SbgEkfNav`` message with a full navigation solution
(``status.solution_mode == 4``), converts that lat/lon to UTM, and broadcasts a
*static* TF chain::

    utm_{zone}_{band}  ->  utm  ->  <frame_prefix>/odom

`utm_{zone}_{band} -> utm` is identity; `utm -> <prefix>/odom` carries the UTM
easting/northing of the datum. ``sbg_to_odom`` then looks up `utm -> odom` once
to recover that offset. The static transforms are re-sent on a timer so late
subscribers (and rosbag splits) still receive them.
"""

from geometry_msgs.msg import TransformStamped
from pingdsp_sbg import sbg_transforms as st
import rclpy
from rclpy.node import Node
from sbg_driver.msg import SbgEkfNav
from tf2_ros import StaticTransformBroadcaster

SOLUTION_MODE_NAV_POSITION = 4


class SbgToOdomInitializer(Node):
    """Locks an odom datum from the first valid SBG fix and serves static TF."""

    def __init__(self):
        super().__init__('sbg_to_odom_initializer')

        self.declare_parameter('frame_prefix', 'pingdsp')
        self.declare_parameter('sbg_nav_topic', 'sbg/ekf_nav')
        self.declare_parameter('update_rate', 1.0)
        self.declare_parameter('require_nav_solution', True)
        self.declare_parameter('verbose', True)

        self.frame_prefix = self.get_parameter('frame_prefix').value
        nav_topic = self.get_parameter('sbg_nav_topic').value
        update_rate = float(self.get_parameter('update_rate').value)
        self.require_nav_solution = bool(
            self.get_parameter('require_nav_solution').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        self.odom_frame = '{}/odom'.format(self.frame_prefix)

        self.origin_set = False
        self.transforms = []

        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Nav fixes are streamed best-effort; keep the latest.
        self.create_subscription(SbgEkfNav, nav_topic, self.nav_callback, 10)

        period = 1.0 / update_rate if update_rate > 0.0 else 1.0
        self.timer = self.create_timer(period, self.republish)

        self.get_logger().info(
            'SBG odom initializer waiting for a fix on "{}" '
            '(require nav solution: {})'.format(
                nav_topic, self.require_nav_solution))

    def nav_callback(self, msg: SbgEkfNav):
        """Lock the datum on the first qualifying fix; ignore the rest."""
        if self.origin_set:
            return
        if (self.require_nav_solution
                and msg.status.solution_mode != SOLUTION_MODE_NAV_POSITION):
            return

        easting, northing, zone, letter = st.latlon_to_utm(
            msg.latitude, msg.longitude)
        utm_zone_frame = st.utm_zone_frame_id(zone, letter)

        now = self.get_clock().now().to_msg()

        zone_to_utm = TransformStamped()
        zone_to_utm.header.stamp = now
        zone_to_utm.header.frame_id = utm_zone_frame
        zone_to_utm.child_frame_id = 'utm'
        zone_to_utm.transform.rotation.w = 1.0

        utm_to_odom = TransformStamped()
        utm_to_odom.header.stamp = now
        utm_to_odom.header.frame_id = 'utm'
        utm_to_odom.child_frame_id = self.odom_frame
        utm_to_odom.transform.translation.x = easting
        utm_to_odom.transform.translation.y = northing
        utm_to_odom.transform.translation.z = 0.0
        utm_to_odom.transform.rotation.w = 1.0

        self.transforms = [zone_to_utm, utm_to_odom]
        self.origin_set = True
        self.static_broadcaster.sendTransform(self.transforms)

        self.get_logger().info(
            'Datum locked at lat={:.7f} lon={:.7f} -> {} '
            'easting={:.2f} northing={:.2f}; broadcasting {} -> utm -> {}'.format(
                msg.latitude, msg.longitude, utm_zone_frame,
                easting, northing, utm_zone_frame, self.odom_frame))

    def republish(self):
        """Re-stamp and re-send the static transforms for late joiners."""
        if not self.origin_set:
            return
        now = self.get_clock().now().to_msg()
        for tf in self.transforms:
            tf.header.stamp = now
        self.static_broadcaster.sendTransform(self.transforms)
        if self.verbose:
            self.get_logger().debug('Republished static datum TF')


def main(args=None):
    rclpy.init(args=args)
    node = SbgToOdomInitializer()
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
