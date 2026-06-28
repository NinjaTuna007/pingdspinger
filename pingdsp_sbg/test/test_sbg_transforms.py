"""Pure-logic tests for pingdsp_sbg.sbg_transforms (no ROS, no sockets)."""

import math

import numpy as np
from pingdsp_sbg import sbg_transforms as st
import pytest
from transforms3d.euler import euler2quat


def _ned_quat_from_yaw(yaw_rad):
    """Build a ROS-ordered (x,y,z,w) NED attitude quaternion for a yaw."""
    w, x, y, z = euler2quat(0.0, 0.0, yaw_rad, axes='sxyz')
    return (x, y, z, w)


def test_identity_ned_points_north_heading_zero():
    enu = st.sbg_quat_to_enu_quat((0.0, 0.0, 0.0, 1.0))
    # Vehicle level + pointing North -> ENU yaw +90 deg.
    assert enu[0] == pytest.approx(0.0, abs=1e-6)
    assert enu[1] == pytest.approx(0.0, abs=1e-6)
    assert abs(enu[2]) == pytest.approx(math.sqrt(0.5), abs=1e-6)
    assert st.enu_quat_to_heading_deg(enu) == pytest.approx(0.0, abs=1e-6)


def test_heading_east_when_ned_yaw_90():
    enu = st.sbg_quat_to_enu_quat(_ned_quat_from_yaw(math.pi / 2))
    assert st.enu_quat_to_heading_deg(enu) == pytest.approx(90.0, abs=1e-3)


def test_heading_wraps_360():
    enu = st.sbg_quat_to_enu_quat(_ned_quat_from_yaw(-math.pi / 2))
    # NED yaw -90 (West) -> compass 270.
    assert st.enu_quat_to_heading_deg(enu) == pytest.approx(270.0, abs=1e-3)


def test_enu_quat_is_normalised():
    enu = st.sbg_quat_to_enu_quat(_ned_quat_from_yaw(0.7))
    assert np.linalg.norm(enu) == pytest.approx(1.0, abs=1e-9)


def test_velocity_north_with_identity_attitude_is_forward():
    vx, vy, vz = st.sbg_velocity_to_body_enu(
        (0.0, 0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
    assert vx == pytest.approx(1.0, abs=1e-6)
    assert vy == pytest.approx(0.0, abs=1e-6)
    assert vz == pytest.approx(0.0, abs=1e-6)


def test_velocity_east_with_yaw90_is_forward():
    # Vehicle heading East (NED yaw 90), ground velocity East -> body forward.
    q = _ned_quat_from_yaw(math.pi / 2)
    vx, vy, vz = st.sbg_velocity_to_body_enu(q, (0.0, 1.0, 0.0))
    assert vx == pytest.approx(1.0, abs=1e-6)
    assert vy == pytest.approx(0.0, abs=1e-6)


def test_gyro_frd_to_flu():
    assert st.sbg_gyro_to_enu((1.0, 2.0, 3.0)) == (1.0, -2.0, -3.0)


def test_wrap_deg_360():
    assert st.wrap_deg_360(370.0) == pytest.approx(10.0)
    assert st.wrap_deg_360(-10.0) == pytest.approx(350.0)
    assert st.wrap_deg_360(0.0) == pytest.approx(0.0)


def test_course_deg():
    # Heading 0 (North), moving straight forward -> course 0.
    assert st.course_deg(0.0, 1.0, 0.0) == pytest.approx(0.0, abs=1e-6)
    # Velocity to the left (vy>0) rotates course CCW.
    assert st.course_deg(0.0, 0.0, 1.0) == pytest.approx(90.0, abs=1e-6)


def test_speed_mps():
    assert st.speed_mps(3.0, 4.0) == pytest.approx(5.0)


def test_latlon_to_utm_and_frame_id():
    easting, northing, zone, letter = st.latlon_to_utm(48.4197, -4.4721)
    assert zone == 30
    assert letter == 'U'
    assert 0 < easting < 1_000_000
    assert st.utm_zone_frame_id(zone, letter) == 'utm_30_U'


def test_euler_matches_quat_attitude():
    """EKF_EULER must yield the same SBG NED quat (and ENU result) as EKF_QUAT.

    The PingDSP Ellipse streams Euler, not quaternion, so the two code paths
    have to agree for non-trivial roll/pitch/yaw.
    """
    roll, pitch, yaw = 0.1, -0.2, 1.3
    q_from_euler = st.sbg_euler_to_quat_xyzw(roll, pitch, yaw)
    w, x, y, z = euler2quat(roll, pitch, yaw, axes='sxyz')
    q_ref = (x, y, z, w)
    # Quaternions may differ by an overall sign; compare absolute components.
    assert np.allclose(np.abs(q_from_euler), np.abs(q_ref), atol=1e-9)

    enu_via_euler = st.sbg_euler_to_enu_quat(roll, pitch, yaw)
    enu_via_quat = st.sbg_quat_to_enu_quat(q_ref)
    assert np.allclose(np.abs(enu_via_euler), np.abs(enu_via_quat), atol=1e-9)


def test_euler_identity_heading_north():
    enu = st.sbg_euler_to_enu_quat(0.0, 0.0, 0.0)
    assert st.enu_quat_to_heading_deg(enu) == pytest.approx(0.0, abs=1e-6)


def test_attitude_matrix_orthonormal():
    enu = st.sbg_quat_to_enu_quat(_ned_quat_from_yaw(1.2))
    from transforms3d.quaternions import quat2mat
    r = quat2mat([enu[3], enu[0], enu[1], enu[2]])
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-9)
    assert np.linalg.det(r) == pytest.approx(1.0, abs=1e-9)
