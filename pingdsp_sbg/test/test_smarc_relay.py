"""In-process check that the SMaRC relay copies nav and lifts altitude."""

import time

from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Odometry
from pingdsp_sbg.smarc_relay import SmarcRelay
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32


def test_relay_passes_degrees_and_altitude():
    owns_init = not rclpy.ok()
    if owns_init:
        rclpy.init()
    namespace = 'smarc_relay_ut'
    relay = SmarcRelay(namespace=namespace)
    probe = Node('smarc_relay_probe', namespace=namespace)
    executor = SingleThreadedExecutor()
    executor.add_node(relay)
    executor.add_node(probe)

    latlon_in = []
    heading_in = []
    course_in = []
    speed_in = []
    altitude_in = []
    odom_in = []
    probe.create_subscription(
        GeoPoint, 'smarc/latlon', latlon_in.append, 10)
    probe.create_subscription(
        Float32, 'smarc/heading', heading_in.append, 10)
    probe.create_subscription(
        Float32, 'smarc/course', course_in.append, 10)
    probe.create_subscription(
        Float32, 'smarc/speed', speed_in.append, 10)
    probe.create_subscription(
        Float32, 'smarc/altitude', altitude_in.append, 10)
    probe.create_subscription(
        Odometry, 'smarc/odom', odom_in.append, 10)

    latlon_pub = probe.create_publisher(GeoPoint, 'latlon', 10)
    heading_pub = probe.create_publisher(Float32, 'heading', 10)
    course_pub = probe.create_publisher(Float32, 'course', 10)
    speed_pub = probe.create_publisher(Float32, 'speed', 10)
    odom_pub = probe.create_publisher(Odometry, 'odom', 10)

    latlon = GeoPoint()
    latlon.latitude = 59.3
    latlon.longitude = 18.1
    latlon.altitude = 12.5
    heading = Float32()
    heading.data = 270.0
    course = Float32()
    course.data = 123.0
    speed = Float32()
    speed.data = 1.25
    odom = Odometry()
    odom.pose.pose.position.x = 4.0
    odom.header.frame_id = 'pingdsp/odom'
    odom.child_frame_id = 'pingdsp/base_link'

    deadline = time.time() + 5.0
    try:
        while time.time() < deadline and not (
                latlon_in and heading_in and course_in and speed_in
                and altitude_in and odom_in):
            latlon_pub.publish(latlon)
            heading_pub.publish(heading)
            course_pub.publish(course)
            speed_pub.publish(speed)
            odom_pub.publish(odom)
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.shutdown()
        relay.destroy_node()
        probe.destroy_node()
        if owns_init and rclpy.ok():
            rclpy.shutdown()

    assert latlon_in, 'smarc/latlon was not published'
    assert latlon_in[-1].latitude == pytest.approx(59.3)
    assert latlon_in[-1].longitude == pytest.approx(18.1)
    assert latlon_in[-1].altitude == pytest.approx(12.5)
    assert heading_in[-1].data == pytest.approx(270.0)
    assert course_in[-1].data == pytest.approx(123.0)
    assert speed_in[-1].data == pytest.approx(1.25)
    assert altitude_in[-1].data == pytest.approx(12.5)
    assert odom_in[-1].pose.pose.position.x == pytest.approx(4.0)
    assert odom_in[-1].header.frame_id == 'pingdsp/odom'
    assert odom_in[-1].child_frame_id == 'pingdsp/base_link'
