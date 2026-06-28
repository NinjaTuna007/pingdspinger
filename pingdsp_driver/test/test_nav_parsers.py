"""Pure-logic tests for pingdsp_driver.nav_parsers (no ROS, no sockets)."""

import math

from pingdsp_driver import nav_parsers as np
import pytest


def test_parse_gpgga_basic():
    lat, lon = np.parse_gpgga(
        '$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47')
    assert lat == pytest.approx(48.1173, abs=1e-4)
    assert lon == pytest.approx(11.5166667, abs=1e-4)


def test_parse_gpgga_south_west_signs():
    lat, lon = np.parse_gpgga('$GPGGA,000000,4807.038,S,01131.000,W,1,08*00')
    assert lat < 0
    assert lon < 0


def test_parse_gpgga_garbage_returns_none():
    assert np.parse_gpgga('not a sentence') is None
    assert np.parse_gpgga('$GPGGA,,,,,') is None


def test_parse_gphdt():
    assert np.parse_gphdt('$GPHDT,123.4,T*0B') == pytest.approx(123.4)
    assert np.parse_gphdt('$GPHDT,,T') is None


def test_parse_gprmc_course():
    rmc = '$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A'
    assert np.parse_gprmc(rmc) == pytest.approx(84.4)


def test_parse_vnycm():
    yaw, pitch, roll = np.parse_vnycm('$VNYCM,+010.000,+002.000,-003.000,1,2,3')
    assert yaw == pytest.approx(10.0)
    assert pitch == pytest.approx(2.0)
    assert roll == pytest.approx(-3.0)


def test_parse_tss1_known():
    # Indices per parser: [14]=roll sign, [15:19]=roll*0.01,
    # [20]=pitch sign, [21:25]=pitch*0.01.
    s = list(':' + 'X' * 24)
    s[14] = '-'
    s[15:19] = list('0150')   # roll = -1.50
    s[20] = '+'
    s[21:25] = list('0075')   # pitch = +0.75
    sentence = ''.join(s)
    roll, pitch = np.parse_tss1(sentence)
    assert roll == pytest.approx(-1.5)
    assert pitch == pytest.approx(0.75)


def test_ned_heading_to_enu_yaw():
    # Heading North (0 deg NED) -> ENU yaw 90 deg.
    assert np.ned_heading_deg_to_enu_yaw_rad(0.0) == pytest.approx(math.pi / 2)
    # Heading East (90 deg) -> ENU yaw 0.
    assert np.ned_heading_deg_to_enu_yaw_rad(90.0) == pytest.approx(0.0)
    # Convergence subtracts.
    assert np.ned_heading_deg_to_enu_yaw_rad(0.0, 0.1) == pytest.approx(
        math.pi / 2 - 0.1)


def test_utm_zone_from_lon():
    assert np.utm_zone_from_lon(-4.47) == 30
    assert np.utm_zone_from_lon(11.5) == 32
    assert np.utm_zone_from_lon(0.0) == 31


def test_latlon_to_utm_roundtrip_zone_lock():
    e1, n1, zone, letter = np.latlon_to_utm(48.4197, -4.4721)
    assert zone == 30
    assert letter == 'U'
    # Forcing the same zone yields identical easting/northing.
    e2, n2, _, _ = np.latlon_to_utm(48.4197, -4.4721, force_zone_number=zone)
    assert e1 == pytest.approx(e2)
    assert n1 == pytest.approx(n2)


def test_meridian_convergence_sign():
    # West of central meridian -> negative convergence in N hemisphere.
    conv = np.meridian_convergence_rad(48.4197, -4.4721, 30)
    assert conv < 0
