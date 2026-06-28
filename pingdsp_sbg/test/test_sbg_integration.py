"""Full-stack integration test for the SBG odom pipeline.

Skipped unless the package is built/sourced and ``sbg_driver`` messages are
importable. Injects synthetic NED SBG messages and asserts the initializer locks
a datum and ``sbg_to_odom`` emits ENU odometry.
"""

import pytest

from sbg_harness import (make_euler, make_imu, make_imu_short, make_nav,
                         make_quat, SBG_AVAILABLE, SBG_MSGS_AVAILABLE, SbgStack,
                         wait_until)

pytestmark = pytest.mark.skipif(
    not (SBG_AVAILABLE and SBG_MSGS_AVAILABLE),
    reason='pingdsp_sbg / sbg_driver not built or sourced')

# Brest-ish (matches the SBG config init position); zone 30U.
LAT, LON = 48.4197, -4.4721


def test_odom_published_from_ned_stream():
    from nav_msgs.msg import Odometry
    from sbg_driver.msg import SbgEkfNav, SbgEkfQuat, SbgImuData

    stack = SbgStack()
    stack.start_nodes(params=['publish_rate:=20.0', 'update_rate:=10.0'])
    probe = stack.start_probe()

    odoms = probe.collect(Odometry, '/odom')
    nav_pub = probe.publisher(SbgEkfNav, '/sbg/ekf_nav')
    quat_pub = probe.publisher(SbgEkfQuat, '/sbg/ekf_quat')
    imu_pub = probe.publisher(SbgImuData, '/sbg/imu_data')

    try:
        def _pump():
            nav_pub.publish(make_nav(LAT, LON, solution_mode=4))
            quat_pub.publish(make_quat())     # identity NED attitude
            imu_pub.publish(make_imu(0.0, 0.0, 0.1))
            return len(odoms) > 0

        assert wait_until(_pump, timeout=25), 'no Odometry published'
        msg = odoms[-1]
        assert msg.header.frame_id == 'pingdsp/odom'
        assert msg.child_frame_id == 'pingdsp/base_link'
        # Datum is the first fix, so the local position starts near origin.
        assert abs(msg.pose.pose.position.x) < 5.0
        assert abs(msg.pose.pose.position.y) < 5.0
        # Identity NED attitude -> ENU yaw ~ +90 deg (qz ~ qw ~ sqrt(0.5)).
        assert msg.pose.pose.orientation.w == pytest.approx(0.707, abs=0.05)
        assert msg.pose.pose.orientation.z == pytest.approx(0.707, abs=0.05)
    finally:
        stack.stop()


def test_odom_published_from_euler_only_stream():
    """The PingDSP Ellipse streams EKF_EULER + IMU_SHORT but no EKF_QUAT.

    With attitude_source 'auto', sbg_to_odom must still produce odometry,
    deriving orientation from EKF_EULER and gyro from IMU_SHORT.
    """
    from nav_msgs.msg import Odometry
    from sbg_driver.msg import SbgEkfEuler, SbgEkfNav, SbgImuShort

    stack = SbgStack()
    stack.start_nodes(params=['publish_rate:=20.0', 'update_rate:=10.0'])
    probe = stack.start_probe()

    odoms = probe.collect(Odometry, '/odom')
    nav_pub = probe.publisher(SbgEkfNav, '/sbg/ekf_nav')
    euler_pub = probe.publisher(SbgEkfEuler, '/sbg/ekf_euler')
    imu_pub = probe.publisher(SbgImuShort, '/sbg/imu_short')

    try:
        def _pump():
            nav_pub.publish(make_nav(LAT, LON, solution_mode=4))
            euler_pub.publish(make_euler(0.0, 0.0, 0.0))  # identity NED
            imu_pub.publish(make_imu_short(0.0, 0.0, 0.1))
            return len(odoms) > 0

        assert wait_until(_pump, timeout=25), 'no Odometry from euler stream'
        msg = odoms[-1]
        assert msg.header.frame_id == 'pingdsp/odom'
        assert msg.child_frame_id == 'pingdsp/base_link'
        # Identity NED attitude -> ENU yaw ~ +90 deg, same as the quat path.
        assert msg.pose.pose.orientation.w == pytest.approx(0.707, abs=0.05)
        assert msg.pose.pose.orientation.z == pytest.approx(0.707, abs=0.05)
        # IMU_SHORT gyro z (+0.1, FRD) -> FLU negates z -> -0.1.
        assert msg.twist.twist.angular.z == pytest.approx(-0.1, abs=0.02)
    finally:
        stack.stop()


def test_initializer_ignores_unsolved_fixes():
    from sbg_driver.msg import SbgEkfNav
    from tf2_msgs.msg import TFMessage

    stack = SbgStack()
    stack.start_nodes(params=['update_rate:=10.0'])
    probe = stack.start_probe()

    static_tf = probe.collect(TFMessage, '/tf_static')
    nav_pub = probe.publisher(SbgEkfNav, '/sbg/ekf_nav')
    try:
        # solution_mode 1 (not a full nav solution) must not lock a datum.
        for _ in range(20):
            nav_pub.publish(make_nav(LAT, LON, solution_mode=1))
            if wait_until(lambda: False, timeout=0.05):
                pass
        assert not wait_until(lambda: len(static_tf) > 0, timeout=3), \
            'datum locked on an unsolved fix'

        # A valid fix then locks the datum and publishes static TF.
        assert wait_until(lambda: (
            nav_pub.publish(make_nav(LAT, LON, solution_mode=4))
            or len(static_tf) > 0), timeout=15), 'datum never locked'
    finally:
        stack.stop()
