#!/usr/bin/env python3
"""
Clicked-point -> NavSatFix bridge for the PingDSP stack.

Foxglove (and RViz) publish ``geometry_msgs/PointStamped`` on ``/clicked_point``
when you click in a 3D scene, but the coordinates are local metres in whatever
the display's fixed frame is (e.g. ``map`` / ``pingdsp/map``). To drop that
click onto a real map you need it as geographic lat/lon.

This node turns a click into a ``sensor_msgs/NavSatFix`` without having to know
the stack's internal datum: it self-calibrates from whatever fix is already on
the wire (``pingdsp/fix`` from the SBG stack, or ``sonar/fix`` from the sonar
driver when there is no SBG) plus the live TF tree:

* the latest NavSatFix gives the geographic position of the platform frame
  (``fix.header.frame_id``) *right now*;
* TF gives that same platform frame's local position in the ENU reference
  frame (by default the clicked point's own frame);
* both the click and the platform are ENU metres in that frame, and the
  drivers build the frame aligned to the UTM grid, so the click's offset from
  the platform (dEast, dNorth) is added to the platform's UTM coordinate and
  projected back to lat/lon.

So the click is geo-referenced relative to the most recent real fix - no datum
parameters, works the same in live and pcap-replay setups.

Published topic: ``clicked_fix`` (sensor_msgs/NavSatFix).
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus

import tf2_ros

from pingdsp_driver import nav_parsers


class ClickedPointToNavSat(Node):
    """Republish ``/clicked_point`` as a geo-referenced NavSatFix."""

    def __init__(self):
        super().__init__('clicked_point_to_navsat')

        self.declare_parameter('input_topic', '/clicked_point')
        self.declare_parameter('output_topic', 'clicked_fix')
        # NavSatFix sources to anchor the geo reference, in priority order. The
        # first one that has published is used. Covers both the SBG stack and
        # the sonar driver's embedded-GPS fix.
        self.declare_parameter('fix_topics', ['pingdsp/fix', 'sonar/fix'])
        # ENU reference frame for the offset maths. Empty => use the clicked
        # point's own frame_id (the Foxglove fixed frame), which the drivers
        # align to the UTM grid. Set this to force a specific frame.
        self.declare_parameter('reference_frame', '')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.fix_topics = list(self.get_parameter('fix_topics').value)
        self.reference_frame = self.get_parameter('reference_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Latest fix used as the geographic anchor.
        self.last_fix = None

        self.fix_subs = []
        for topic in self.fix_topics:
            self.fix_subs.append(
                self.create_subscription(
                    NavSatFix, topic,
                    lambda msg, t=topic: self._fix_callback(msg, t), 10))

        self.click_sub = self.create_subscription(
            PointStamped, self.input_topic, self._click_callback, 10)
        self.fix_pub = self.create_publisher(
            NavSatFix, self.output_topic, 10)

        self.get_logger().info('ClickedPoint -> NavSatFix bridge up')
        self.get_logger().info(f'  Input:  {self.input_topic}')
        self.get_logger().info(f'  Output: {self.output_topic}')
        self.get_logger().info(f'  Fix anchors: {self.fix_topics}')

    def _fix_callback(self, msg: NavSatFix, topic: str):
        """Keep the most recent usable fix as the geo anchor."""
        # Ignore fixes that carry no actual solution (status NO_FIX) so we do
        # not anchor on a placeholder (0, 0).
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        self.last_fix = msg

    def _click_callback(self, msg: PointStamped):
        """Geo-reference one clicked point and publish it as a NavSatFix."""
        if self.last_fix is None:
            self.get_logger().warn(
                'Clicked point ignored: no NavSatFix received yet on '
                f'{self.fix_topics} to anchor the geo reference.')
            return

        ref_frame = self.reference_frame or msg.header.frame_id
        if not ref_frame:
            self.get_logger().warn(
                'Clicked point has no frame_id and no reference_frame set.')
            return

        platform_frame = self.last_fix.header.frame_id

        # Click coordinates in the ENU reference frame.
        try:
            click = self._point_in_frame(msg, ref_frame)
        except Exception as e:  # noqa: BLE001 - TF errors -> warn + skip
            self.get_logger().warn(
                f'Could not transform clicked point into {ref_frame}: {e}')
            return

        # Platform position in the same ENU reference frame (= translation of
        # the transform ref_frame -> platform_frame).
        try:
            tf = self.tf_buffer.lookup_transform(
                ref_frame, platform_frame, Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(
                f'No TF {ref_frame} -> {platform_frame} to locate the '
                f'platform: {e}')
            return
        px = tf.transform.translation.x
        py = tf.transform.translation.y

        # Offset of the click from the platform, in ENU metres.
        d_east = click[0] - px
        d_north = click[1] - py

        # Anchor: platform fix -> UTM, add the offset, project back to lat/lon.
        # Guard the projection so an off-grid click (or a degenerate fix) logs a
        # warning instead of taking the node down.
        try:
            easting, northing, zone, letter = nav_parsers.latlon_to_utm(
                self.last_fix.latitude, self.last_fix.longitude)
            lat, lon = nav_parsers.utm_to_latlon(
                easting + d_east, northing + d_north, zone, zone_letter=letter)
        except Exception as e:  # noqa: BLE001 - bad click -> warn + skip
            self.get_logger().warn(
                f'Could not geo-reference clicked point '
                f'({d_east:+.1f} E, {d_north:+.1f} N m from platform): {e}')
            return

        out = NavSatFix()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = ref_frame
        status = NavSatStatus()
        status.status = NavSatStatus.STATUS_FIX
        status.service = NavSatStatus.SERVICE_GPS
        out.status = status
        out.latitude = lat
        out.longitude = lon
        out.altitude = float(self.last_fix.altitude) + float(click[2])
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(out)
        self.get_logger().info(
            f'Clicked point ({d_east:+.1f} E, {d_north:+.1f} N m from '
            f'platform) -> lat {lat:.7f}, lon {lon:.7f}')

    def _point_in_frame(self, msg: PointStamped, frame: str):
        """Return the click as (x, y, z) in ``frame`` (ENU metres)."""
        if msg.header.frame_id == frame:
            p = msg.point
            return (p.x, p.y, p.z)
        # Different frame: transform via TF. tf2_geometry_msgs registers the
        # PointStamped transform with the buffer on import.
        import tf2_geometry_msgs  # noqa: F401 - registers do_transform
        out = self.tf_buffer.transform(
            msg, frame, timeout=Duration(seconds=0.2))
        return (out.point.x, out.point.y, out.point.z)


def main(args=None):
    rclpy.init(args=args)
    node = ClickedPointToNavSat()
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
