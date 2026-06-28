#!/usr/bin/env python3
"""Pure, ROS-free SBG (NED) -> ROS (ENU) conversion helpers.

The SBG Ellipse driver here runs with ``use_enu: false`` so every message is in
the SBG/NED convention. evolo_common's ``sbg_to_odom`` (C++/Eigen) turns that
into a ROS ENU odometry + TF tree. This module re-implements exactly that math
in Python using ``transforms3d`` (not ``tf_transformations``), kept free of any
ROS imports so it can be unit tested in isolation.

Conventions (mirrors evolo):

* ``R_NED_TO_ENU = [[0,1,0],[1,0,0],[0,0,-1]]`` (swap N/E, flip D).
* ``R_SBG_TO_ROS = Rx(pi)`` (body FRD -> FLU).
* attitude:  ``R_enu = R_NED_TO_ENU @ R_sbg @ R_SBG_TO_ROS``.
* velocity:  rotate NED ground velocity into body via the SBG quaternion's
  inverse, then negate y, z to land in FLU body.
* gyro:      negate y, z (FRD -> FLU).
* heading:   ``90 - yaw_enu_deg`` wrapped to [0, 360).

transforms3d quaternion order is ``(w, x, y, z)``; ROS ``geometry_msgs/Quaternion``
is ``(x, y, z, w)``. Helpers here take/return ROS ``(x, y, z, w)`` tuples and do
the reordering internally.
"""

import math

import numpy as np
from transforms3d.euler import euler2mat, quat2euler
from transforms3d.quaternions import mat2quat, quat2mat

# North-East-Down -> East-North-Up.
R_NED_TO_ENU = np.array([
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
])

# SBG body (Forward-Right-Down) -> ROS body (Forward-Left-Up) = rotate pi about X.
R_SBG_TO_ROS = euler2mat(math.pi, 0.0, 0.0, axes='sxyz')


def _ros_to_t3d(q_xyzw):
    """ROS (x, y, z, w) -> transforms3d (w, x, y, z) numpy array."""
    x, y, z, w = q_xyzw
    return np.array([w, x, y, z])


def _t3d_to_ros(q_wxyz):
    """transforms3d (w, x, y, z) -> ROS (x, y, z, w) tuple."""
    w, x, y, z = q_wxyz
    return (x, y, z, w)


def _r_sbg_to_enu_quat(r_sbg):
    """Map an SBG-body->NED rotation matrix to a ROS ENU quaternion (xyzw)."""
    r_enu = R_NED_TO_ENU @ r_sbg @ R_SBG_TO_ROS
    q_wxyz = mat2quat(r_enu)
    n = np.linalg.norm(q_wxyz)
    if n > 0.0:
        q_wxyz = q_wxyz / n
    return _t3d_to_ros(q_wxyz)


def sbg_quat_to_enu_quat(q_xyzw):
    """Convert an SBG NED attitude quaternion to a ROS ENU quaternion.

    Args:
        q_xyzw: SBG attitude quaternion as ROS-ordered (x, y, z, w).

    Returns:
        ROS-ordered (x, y, z, w) ENU attitude quaternion (normalised).
    """
    return _r_sbg_to_enu_quat(quat2mat(_ros_to_t3d(q_xyzw)))


def sbg_euler_to_quat_xyzw(roll, pitch, yaw):
    """Build the SBG NED attitude quaternion from EKF Euler angles.

    The SBG ``SbgEkfEuler`` log carries (roll, pitch, yaw) in the NED body
    convention, encoding the body->NED rotation as
    ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``. This returns the matching attitude
    quaternion in ROS order (x, y, z, w), i.e. exactly what ``SbgEkfQuat`` would
    have provided, so the rest of the pipeline is identical whether attitude
    arrives as a quaternion or as Euler angles.

    Args:
        roll: roll about the NED X axis [rad].
        pitch: pitch about the NED Y axis [rad].
        yaw: yaw about the NED down axis [rad].

    Returns:
        SBG NED attitude quaternion as ROS-ordered (x, y, z, w).
    """
    # Static-axes 'sxyz' composes as Rz(yaw) @ Ry(pitch) @ Rx(roll).
    r_sbg = euler2mat(roll, pitch, yaw, axes='sxyz')
    q_wxyz = mat2quat(r_sbg)
    n = np.linalg.norm(q_wxyz)
    if n > 0.0:
        q_wxyz = q_wxyz / n
    return _t3d_to_ros(q_wxyz)


def sbg_euler_to_enu_quat(roll, pitch, yaw):
    """Convert SBG NED Euler angles directly to a ROS ENU quaternion (xyzw)."""
    return _r_sbg_to_enu_quat(euler2mat(roll, pitch, yaw, axes='sxyz'))


def sbg_velocity_to_body_enu(q_xyzw, vel_ned_xyz):
    """Rotate NED ground velocity into the ROS FLU body frame.

    Mirrors evolo: ``v_body = q_ned.inverse() * v_global`` then negate y, z.

    Args:
        q_xyzw: SBG attitude quaternion (x, y, z, w), maps NED->body.
        vel_ned_xyz: ground velocity in NED (vx_north, vy_east, vz_down).

    Returns:
        (vx, vy, vz) linear velocity in the FLU body frame.
    """
    # Rotating a vector by the inverse of q is R(q)^T applied to the vector.
    r_ned_to_body = quat2mat(_ros_to_t3d(q_xyzw))
    v_body = r_ned_to_body.T @ np.asarray(vel_ned_xyz, dtype=float)
    return (float(v_body[0]), float(-v_body[1]), float(-v_body[2]))


def sbg_gyro_to_enu(gyro_xyz):
    """Convert SBG (FRD) body angular rates to ROS (FLU) by negating y, z."""
    gx, gy, gz = gyro_xyz
    return (float(gx), float(-gy), float(-gz))


def enu_yaw_rad(q_xyzw):
    """Return the ENU yaw (rad) of a ROS-ordered quaternion (axes='sxyz')."""
    roll, pitch, yaw = quat2euler(_ros_to_t3d(q_xyzw), axes='sxyz')
    return float(yaw)


def wrap_deg_360(angle_deg):
    """Wrap an angle in degrees to [0, 360)."""
    a = math.fmod(angle_deg, 360.0)
    if a < 0.0:
        a += 360.0
    return a


def enu_quat_to_heading_deg(q_xyzw):
    """Compass heading (deg, [0,360)) from a ROS ENU quaternion.

    Uses the evolo convention ``heading = 90 - yaw_enu_deg``.
    """
    yaw_deg = math.degrees(enu_yaw_rad(q_xyzw))
    return wrap_deg_360(90.0 - yaw_deg)


def course_deg(heading_deg, vx, vy):
    """Course over ground (deg, [0,360)) from heading and body velocity.

    Mirrors evolo: ``course = heading + (180/pi) * atan2(vy, vx)``.
    """
    return wrap_deg_360(heading_deg + math.degrees(math.atan2(vy, vx)))


def speed_mps(vx, vy):
    """Horizontal speed magnitude from body velocity components."""
    return float(math.hypot(vx, vy))


def latlon_to_utm(latitude_deg, longitude_deg):
    """Convert WGS84 lat/lon (deg) to UTM via the ``utm`` library.

    Returns:
        (easting_m, northing_m, zone_number, zone_letter).
    """
    import utm
    easting, northing, zone_number, zone_letter = utm.from_latlon(
        latitude_deg, longitude_deg)
    return (float(easting), float(northing), int(zone_number), str(zone_letter))


def utm_zone_frame_id(zone_number, zone_letter):
    """TF frame name for a UTM zone, e.g. ``utm_33_V`` (matches evolo)."""
    return 'utm_{}_{}'.format(zone_number, zone_letter)
