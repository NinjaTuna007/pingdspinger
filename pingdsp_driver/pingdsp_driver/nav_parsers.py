#!/usr/bin/env python3
"""
Pure parsers for the navigation sentences embedded in the 3DSS-DX ASCII stream.

The 3DSS-DX multiplexes NMEA-0183 / TSS1 / VectorNav sentences into each ping's
ASCII block. The driver needs roll/pitch/yaw and lat/lon out of these. Keeping
the parsing here - as side-effect-free functions returning plain numbers - lets
us unit test the (fiddly, index-based) sentence handling without standing up a
ROS node or a socket (see ``test/test_nav_parsers.py``).

Angle conventions:
* Parsers return angles in **degrees**, exactly as they appear on the wire
  (NED / true-north for headings). Conversion to ENU/grid radians is the
  caller's job - see :func:`ned_heading_deg_to_enu_yaw_rad`.
* lat/lon are returned in signed decimal degrees.
"""

import math
from typing import Optional, Tuple

import numpy as np


def parse_vnycm(sentence: str) -> Optional[Tuple[float, float, float]]:
    """
    Parse a VectorNav ``$VNYCM`` sentence (internal MRU).

    Format: ``$VNYCM,+YYY.YYY,+PPP.PPP,+RRR.RRR,...*cc``

    Returns:
        ``(yaw_deg, pitch_deg, roll_deg)`` (true-north yaw) or None.
    """
    try:
        parts = sentence.split(',')
        if len(parts) >= 4:
            return float(parts[1]), float(parts[2]), float(parts[3])
    except (ValueError, IndexError):
        return None
    return None


def parse_tss1(sentence: str) -> Optional[Tuple[float, float]]:
    """
    Parse a TSS1 attitude sentence.

    Format: ``:XXAAAASMHHHHQMRRRRSMPPPP`` where RRRR/PPPP are roll/pitch in
    units of 0.01 degree, each preceded by a sign character.

    Returns:
        ``(roll_deg, pitch_deg)`` or None.
    """
    try:
        if len(sentence) >= 25 and sentence.startswith(':'):
            roll = float(sentence[15:19]) * 0.01
            if sentence[14] == '-':
                roll = -roll
            pitch = float(sentence[21:25]) * 0.01
            if sentence[20] == '-':
                pitch = -pitch
            return roll, pitch
    except (ValueError, IndexError):
        return None
    return None


def _nmea_lat(value: str, hemisphere: str) -> Optional[float]:
    """Convert NMEA ddmm.mmmm + N/S to signed decimal degrees."""
    if not value or not hemisphere:
        return None
    deg = float(value[:2])
    minutes = float(value[2:])
    lat = deg + minutes / 60.0
    return -lat if hemisphere.upper() == 'S' else lat


def _nmea_lon(value: str, hemisphere: str) -> Optional[float]:
    """Convert NMEA dddmm.mmmm + E/W to signed decimal degrees."""
    if not value or not hemisphere:
        return None
    deg = float(value[:3])
    minutes = float(value[3:])
    lon = deg + minutes / 60.0
    return -lon if hemisphere.upper() == 'W' else lon


def parse_gpgga(sentence: str) -> Optional[Tuple[float, float]]:
    """
    Parse a ``$GPGGA`` / ``$GNGGA`` GPS fix sentence.

    Returns:
        ``(latitude_deg, longitude_deg)`` or None.
    """
    try:
        parts = sentence.split(',')
        if len(parts) >= 6:
            lat = _nmea_lat(parts[2], parts[3])
            lon = _nmea_lon(parts[4], parts[5])
            if lat is not None and lon is not None:
                return lat, lon
    except (ValueError, IndexError):
        return None
    return None


def parse_gphdt(sentence: str) -> Optional[float]:
    """
    Parse a ``$GPHDT`` / ``$GNHDT`` true-heading sentence.

    Format: ``$GPHDT,x.x,T*cc``.

    Returns:
        Heading in degrees relative to true north, or None.
    """
    try:
        parts = sentence.split(',')
        if len(parts) >= 2 and parts[1]:
            return float(parts[1])
    except (ValueError, IndexError):
        return None
    return None


def parse_gprmc(sentence: str) -> Optional[float]:
    """
    Parse the track-made-good (course) from a ``$GPRMC`` / ``$GNRMC`` sentence.

    Field 8 is course over ground in degrees relative to true north.

    Returns:
        Course in degrees, or None.
    """
    try:
        parts = sentence.split(',')
        if len(parts) >= 9 and parts[8]:
            return float(parts[8])
    except (ValueError, IndexError):
        return None
    return None


def ned_heading_deg_to_enu_yaw_rad(heading_ned_deg: float,
                                   meridian_convergence_rad: float = 0.0
                                   ) -> float:
    """
    Convert an NED true-north heading (deg) to an ENU grid yaw (rad).

    NMEA headings are NED (0 deg = North, clockwise). ROS uses ENU
    (0 rad = East, counter-clockwise): ``yaw_enu = 90 - heading_ned``. The
    meridian convergence then rotates true north onto grid (UTM) north.

    Args:
        heading_ned_deg: heading in degrees relative to true north.
        meridian_convergence_rad: grid convergence at the location (rad).

    Returns:
        Grid yaw in radians (ENU).
    """
    heading_enu_deg = 90.0 - heading_ned_deg
    return math.radians(heading_enu_deg) - meridian_convergence_rad


def utm_zone_from_lon(longitude_deg: float) -> int:
    """Return the UTM zone number (1-60) for a longitude in degrees."""
    return int((longitude_deg + 180) / 6) + 1


def latlon_to_utm(latitude_deg: float, longitude_deg: float,
                  force_zone_number: Optional[int] = None
                  ) -> Tuple[float, float, int, str]:
    """Convert WGS84 lat/lon (deg) to UTM via the ``utm`` library.

    Returns (easting_m, northing_m, zone_number, zone_letter). Kept here so the
    driver and tests share one UTM implementation (matches ``pingdsp_sbg``).

    Pass ``force_zone_number`` to keep projecting into a fixed zone (locked on
    the first fix) so easting/northing stay continuous across a zone boundary.
    """
    import utm
    if force_zone_number is not None:
        easting, northing, zone_number, zone_letter = utm.from_latlon(
            latitude_deg, longitude_deg, force_zone_number=force_zone_number)
    else:
        easting, northing, zone_number, zone_letter = utm.from_latlon(
            latitude_deg, longitude_deg)
    return (float(easting), float(northing), int(zone_number), str(zone_letter))


def meridian_convergence_rad(latitude_deg: float, longitude_deg: float,
                             zone: Optional[int] = None) -> float:
    """
    Grid (meridian) convergence at a point, in radians.

    This is the angle between true north and UTM grid north. Positive means
    grid north is east of true north.
    """
    if zone is None:
        zone = utm_zone_from_lon(longitude_deg)
    central_meridian = (zone - 1) * 6 - 180 + 3
    return float(np.arctan(
        np.tan(np.radians(longitude_deg - central_meridian))
        * np.sin(np.radians(latitude_deg))))
