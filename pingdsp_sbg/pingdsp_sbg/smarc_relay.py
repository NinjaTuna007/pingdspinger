#!/usr/bin/env python3
"""
Republish PingDSP nav onto the SMaRC names ``waraps_vehicle`` reads.

``sbg_to_odom`` already publishes geographic position, compass heading,
course, speed, and odometry. The WARA-PS agent does not subscribe to those
names. It reads ``smarc/latlon``, ``smarc/heading``, ``smarc/course``,
``smarc/speed``, ``smarc/odom``, and a separate ``smarc/altitude`` float
(the GeoPoint's altitude is ignored by ``GenericSMaRCVehicle``).

This node copies those messages. It does not publish TF. Heading and course
stay in degrees, which is what ``sbg_to_odom`` and SAM already use.
"""

from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SmarcRelay(Node):
    """Copy PingDSP nav topics onto ``smarc/*`` in the same namespace."""

    def __init__(self, **kwargs):
        super().__init__('smarc_relay', **kwargs)

        self.declare_parameter('latlon_topic', 'latlon')
        self.declare_parameter('heading_topic', 'heading')
        self.declare_parameter('course_topic', 'course')
        self.declare_parameter('speed_topic', 'speed')
        self.declare_parameter('odom_topic', 'odom')

        latlon_topic = self.get_parameter('latlon_topic').value
        heading_topic = self.get_parameter('heading_topic').value
        course_topic = self.get_parameter('course_topic').value
        speed_topic = self.get_parameter('speed_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self._latlon_pub = self.create_publisher(GeoPoint, 'smarc/latlon', 10)
        self._altitude_pub = self.create_publisher(
            Float32, 'smarc/altitude', 10)
        self._heading_pub = self.create_publisher(
            Float32, 'smarc/heading', 10)
        self._course_pub = self.create_publisher(Float32, 'smarc/course', 10)
        self._speed_pub = self.create_publisher(Float32, 'smarc/speed', 10)
        self._odom_pub = self.create_publisher(Odometry, 'smarc/odom', 10)

        self.create_subscription(
            GeoPoint, latlon_topic, self._on_latlon, 10)
        self.create_subscription(
            Float32, heading_topic, self._on_heading, 10)
        self.create_subscription(
            Float32, course_topic, self._on_course, 10)
        self.create_subscription(
            Float32, speed_topic, self._on_speed, 10)
        self.create_subscription(
            Odometry, odom_topic, self._on_odom, 10)

        self.get_logger().info(
            'SMaRC relay: %s -> smarc/latlon (+ smarc/altitude), '
            '%s -> smarc/heading, %s -> smarc/course, '
            '%s -> smarc/speed, %s -> smarc/odom'
            % (latlon_topic, heading_topic, course_topic,
               speed_topic, odom_topic))

    def _on_latlon(self, msg):
        self._latlon_pub.publish(msg)
        altitude = Float32()
        altitude.data = float(msg.altitude)
        self._altitude_pub.publish(altitude)

    def _on_heading(self, msg):
        self._heading_pub.publish(msg)

    def _on_course(self, msg):
        self._course_pub.publish(msg)

    def _on_speed(self, msg):
        self._speed_pub.publish(msg)

    def _on_odom(self, msg):
        self._odom_pub.publish(msg)


def main(args=None):
    """Run the SMaRC nav relay until shutdown."""
    rclpy.init(args=args)
    node = SmarcRelay()
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
