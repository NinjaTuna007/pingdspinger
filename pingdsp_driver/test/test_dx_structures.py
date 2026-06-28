"""Pure-logic tests for pingdsp_driver.dx_structures via synthetic frames."""

from dx_frame_factory import build_dx_data, build_dx_frame
from pingdsp_driver.dx_structures import DX_PREAMBLE, DxData, DxHeader
import pytest


def test_header_roundtrip():
    frame = build_dx_frame(ping_id=7)
    header = DxHeader.from_bytes(frame[:DxHeader.SIZE])
    assert header.is_valid()
    assert header.preamble == DX_PREAMBLE
    assert header.data_count == len(frame) - DxHeader.SIZE


def test_header_rejects_bad_preamble():
    header = DxHeader.from_bytes(b'\x00' * DxHeader.SIZE)
    assert not header.is_valid()


def test_dxdata_scalar_fields():
    data = DxData.from_bytes(build_dx_data(
        ping_id=99, sample_rate=48000.0, ping_rate=12.0, range_m=75.0))
    assert data.ping_id == 99
    assert data.sample_rate_hz == pytest.approx(48000.0)
    assert data.ping_rate_hz == pytest.approx(12.0)
    assert data.parameters.range_m == pytest.approx(75.0)


def test_ascii_sentences_extracted():
    data = DxData.from_bytes(build_dx_data(
        ascii_sentences=['$GPHDT,45.0,T*0B', '$VNYCM,+1,+2,+3']))
    text = data.get_ascii_sentences()
    assert '$GPHDT,45.0,T*0B' in text
    assert '$VNYCM,+1,+2,+3' in text


def test_sidescan_samples_extracted():
    data = DxData.from_bytes(build_dx_data(
        port_sidescan=[1.0, 2.0, 3.0],
        starboard_sidescan=[4.0, 5.0]))
    assert list(data.get_port_sidescan()) == [1.0, 2.0, 3.0]
    assert list(data.get_starboard_sidescan()) == [4.0, 5.0]


def test_bathymetry_points_and_xyz():
    data = DxData.from_bytes(build_dx_data(
        port_bathy=[(10.0, -0.2, 100.0), (11.0, -0.25, 90.0)],
        starboard_bathy=[(12.0, 0.2, 80.0)]))
    assert len(data.get_port_bathymetry()) == 2
    assert len(data.get_starboard_bathymetry()) == 1
    xyz = data.get_all_bathymetry_xyz(transducer_tilt_deg=30.0)
    assert xyz.shape == (3, 3)
    # Port points get +Y, starboard -Y (horizontal component).
    assert xyz[0, 1] > 0
    assert xyz[2, 1] < 0


def test_empty_sections_are_empty():
    data = DxData.from_bytes(build_dx_data())
    assert data.get_ascii_sentences() == ''
    assert len(data.get_port_sidescan()) == 0
    assert data.get_all_bathymetry_xyz().shape == (0, 3)
