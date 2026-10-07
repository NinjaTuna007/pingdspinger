"""Pure-logic tests for pingdsp_driver.dx_structures via synthetic frames."""

from dx_frame_factory import build_dx_data, build_dx_frame
import numpy as np
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
    assert data.get_port_sidescan().dtype == np.float32
    assert data.get_port_sidescan().flags['C_CONTIGUOUS']


def test_sidescan_ranges_are_c_over_2fs_grid():
    # 1485 m/s @ 56.25 kHz is the Orebro/Asko configuration: 0.0132 m/sample.
    data = DxData.from_bytes(build_dx_data(
        port_sidescan=[0.0] * 2848, starboard_sidescan=[0.0] * 10,
        sample_rate=56250.0, sv_bulk=1485.0))
    assert data.sidescan_bin_spacing_m() == pytest.approx(0.0132)
    r = data.get_port_sidescan_ranges()
    assert r.shape == (2848,)
    assert r[0] == 0.0
    assert r[1] == pytest.approx(0.0132, abs=1e-6)
    assert r[-1] == pytest.approx(2847 * 0.0132, rel=1e-5)
    assert data.get_starboard_sidescan_ranges().shape == (10,)
    assert data.check_sidescan_grid() == []


def test_sidescan_grid_check_tolerates_head_rounding():
    # Asko capture: head emitted 0.01315 steps for c/(2fs)=0.0131556 (0.04 %).
    data = DxData.from_bytes(build_dx_data(
        port_sidescan=[0.0] * 11536, sample_rate=56250.0, sv_bulk=1480.0,
        sidescan_step_m=0.01315))
    assert data.check_sidescan_grid() == []


def test_sidescan_grid_check_flags_decimation_and_offset():
    # Decimated by 2: step is twice c/(2fs).
    data = DxData.from_bytes(build_dx_data(
        port_sidescan=[0.0] * 100, sample_rate=56250.0, sv_bulk=1485.0,
        sidescan_step_m=2 * 0.0132))
    problems = data.check_sidescan_grid()
    assert len(problems) == 1 and problems[0].startswith('port sidescan step')
    # Start offset of 1 m (starboard only populated).
    data = DxData.from_bytes(build_dx_data(
        starboard_sidescan=[0.0] * 100, sample_rate=56250.0, sv_bulk=1485.0,
        sidescan_start_m=1.0))
    problems = data.check_sidescan_grid()
    assert len(problems) == 1
    assert problems[0].startswith('starboard sidescan first sample at 1.0000')


def test_sidescan_grid_check_skips_without_inputs_or_samples():
    # No sidescan samples -> nothing to check.
    assert DxData.from_bytes(build_dx_data()).check_sidescan_grid() == []
    # No sample_rate -> cannot evaluate, must not raise or flag.
    data = DxData.from_bytes(build_dx_data(
        port_sidescan=[0.0] * 10, sample_rate=0.0))
    assert data.sidescan_bin_spacing_m() == 0.0
    assert data.check_sidescan_grid() == []


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


def test_reported_transducer_tilts_negated_to_driver_convention():
    """Sonar reports positive = downward; the geometry here wants negative."""
    data = DxData.from_bytes(build_dx_data(port_angle=20.0, stbd_angle=20.0))
    assert data.reported_transducer_tilts(-99.0) == (-20.0, -20.0)


def test_reported_transducer_tilts_keep_sides_independent():
    """An asymmetric mount must survive; one tilt param cannot express it."""
    data = DxData.from_bytes(build_dx_data(port_angle=20.0, stbd_angle=25.0))
    assert data.reported_transducer_tilts(-99.0) == (-20.0, -25.0)


def test_reported_transducer_tilts_fall_back_when_implausible():
    """A zeroed or absurd field must not silently flatten the swath."""
    data = DxData.from_bytes(build_dx_data(port_angle=0.0, stbd_angle=1e30))
    assert data.reported_transducer_tilts(-20.0) == (-20.0, -20.0)


def test_bathymetry_xyzi_applies_tilt_per_side():
    """A steeper starboard tilt must push only starboard points deeper."""
    data = DxData.from_bytes(build_dx_data(
        port_bathy=[(10.0, 0.0, 100.0)],
        starboard_bathy=[(10.0, 0.0, 80.0)]))
    sym = data.get_all_bathymetry_xyzi(-20.0)
    asym = data.get_all_bathymetry_xyzi(
        -20.0, tilt_port_deg=-20.0, tilt_stbd_deg=-40.0)
    # port row unchanged, starboard row deeper (more negative z)
    assert asym[0, 2] == sym[0, 2]
    assert asym[1, 2] < sym[1, 2]


def test_bathymetry_xyziq_matches_loop_and_carries_count():
    """Vectorised path == per-point path; reserved1 uint32 -> quality."""
    data = DxData.from_bytes(build_dx_data(
        port_bathy=[(10.0, -0.2, 100.0, 7), (11.0, -0.25, 90.0, 206)],
        starboard_bathy=[(12.0, 0.2, 80.0, 1)]))
    xyzi = data.get_all_bathymetry_xyzi(
        -20.0, tilt_port_deg=-20.0, tilt_stbd_deg=-25.0)
    xyziq = data.get_all_bathymetry_xyziq(-20.0, -25.0)
    assert xyziq.shape == (3, 5)
    assert np.allclose(xyziq[:, :4], xyzi, atol=1e-5)
    assert xyziq[:, 4].tolist() == [7.0, 206.0, 1.0]


def test_bathymetry_xyziq_drops_corrupt_points():
    data = DxData.from_bytes(build_dx_data(
        port_bathy=[(10.0, 0.0, 100.0), (1e19, 0.0, 1.0), (float('nan'), 0.0, 1.0)],
        starboard_bathy=[(12.0, 0.0, 80.0)]))
    xyziq = data.get_all_bathymetry_xyziq(-20.0, -20.0)
    assert xyziq.shape == (2, 5)
    assert xyziq[0, 1] > 0 and xyziq[1, 1] < 0


def test_sidescan3d_points_and_xyzis():
    """3D set uses the same geometry as bathymetry; SNR rides along."""
    pts_port = [(10.0, -0.2, 100.0, 37.2), (10.0, -0.3, 50.0, 20.0)]
    pts_stbd = [(12.0, 0.2, 80.0, 30.0)]
    data = DxData.from_bytes(build_dx_data(
        port_sidescan3d=pts_port, starboard_sidescan3d=pts_stbd,
        port_bathy=[(10.0, -0.2, 100.0)], starboard_bathy=[(12.0, 0.2, 80.0)]))
    port = data.get_port_sidescan3d()
    assert port.shape == (2, 4)
    assert port[0].tolist() == pytest.approx([10.0, -0.2, 100.0, 37.2], abs=1e-5)
    assert data.get_starboard_sidescan3d().shape == (1, 4)

    ss3d = data.get_all_sidescan3d_xyzis(-20.0, -25.0)
    bathy = data.get_all_bathymetry_xyziq(-20.0, -25.0)
    assert ss3d.shape == (3, 5)
    # Point shared by both sections lands at the same xyz.
    assert np.allclose(ss3d[0, :4], bathy[0, :4], atol=1e-5)
    assert np.allclose(ss3d[2, :4], bathy[1, :4], atol=1e-5)
    assert ss3d[:, 4].tolist() == pytest.approx([37.2, 20.0, 30.0], abs=1e-5)
    assert ss3d[0, 1] > 0 and ss3d[2, 1] < 0


def test_settings_snapshot_keys_and_values():
    data = DxData.from_bytes(build_dx_data(
        range_m=75.0, sv_bulk=1485.0, sample_rate=56250.0))
    snap = data.settings_snapshot()
    assert snap['range'] == pytest.approx(75.0)
    assert snap['sound_velocity_bulk'] == pytest.approx(1485.0)
    assert snap['sample_rate'] == pytest.approx(56250.0)
    assert snap['sidescan_bin_spacing'] == pytest.approx(1485.0 / (2 * 56250.0))
    assert snap['sonar_id'] == 'TEST-3DSS'
    assert snap['port_transmit_power'] == 80
    assert isinstance(snap['port_transmit_pulse'], str)
    # Changing a setting changes the snapshot; same settings compare equal.
    same = DxData.from_bytes(build_dx_data(
        ping_id=2, range_m=75.0, sv_bulk=1485.0, sample_rate=56250.0))
    other = DxData.from_bytes(build_dx_data(range_m=50.0, sv_bulk=1485.0,
                                            sample_rate=56250.0))
    assert same.settings_snapshot() == snap
    assert other.settings_snapshot() != snap


def _clean_kwargs(ping_id=1):
    return {
        'ping_id': ping_id,
        'port_sidescan': [1.0] * 64, 'starboard_sidescan': [2.0] * 64,
        'port_sidescan3d': [(10.0, -0.2, 100.0, 37.2)],
        'starboard_sidescan3d': [(12.0, 0.2, 80.0, 30.0)],
        'port_bathy': [(10.0, -0.2, 100.0, 7)],
        'starboard_bathy': [(12.0, 0.2, 80.0, 1)],
        'range_m': 50.0}


def test_frame_problems_clean_frame_is_clean():
    assert DxData.from_bytes(build_dx_data(**_clean_kwargs())).frame_problems() == []
    # Empty sections are fine too.
    assert DxData.from_bytes(build_dx_data()).frame_problems() == []


def test_frame_problems_detects_spliced_frame():
    """Next frame's bytes at this frame's advertised length -> preamble in body."""
    a = build_dx_data(**_clean_kwargs(1))
    b = build_dx_frame(**_clean_kwargs(2))
    cut = len(a) - 200
    spliced = a[:cut] + b[:len(a) - cut]
    assert len(spliced) == len(a)
    problems = DxData.from_bytes(spliced).frame_problems()
    assert len(problems) == 1 and 'preamble inside body' in problems[0]


def test_frame_problems_detects_garbage_points_and_grid():
    bad = DxData.from_bytes(build_dx_data(
        port_bathy=[(1e32, 6.4e37, 1.0, 4250000000)],
        starboard_bathy=[(float('nan'), 0.0, 1.0)],
        port_sidescan3d=[(5.0, 0.1, 1.0, float('inf'))],
        range_m=50.0))
    msgs = ' '.join(bad.frame_problems())
    assert 'port bathymetry range outside [0, 75] m' in msgs
    assert 'port bathymetry angle beyond' in msgs
    assert 'starboard bathymetry has non-finite values' in msgs
    assert 'port sidescan3d has non-finite values' in msgs
    assert 'port bathymetry quality count 4.25e+09' in msgs

    decimated = DxData.from_bytes(build_dx_data(
        port_sidescan=[1.0] * 64, sidescan_step_m=0.5))
    assert any('sidescan step' in p for p in decimated.frame_problems())

    inf_amp = DxData.from_bytes(build_dx_data(
        port_sidescan=[1.0] * 10 + [float('inf')]))
    assert 'port sidescan has non-finite amplitudes' in inf_amp.frame_problems()


def test_frame_problems_allows_points_slightly_past_range_setting():
    ok = DxData.from_bytes(build_dx_data(
        port_bathy=[(55.0, -0.2, 100.0)], range_m=50.0))
    assert ok.frame_problems() == []


def test_empty_sections_are_empty():
    data = DxData.from_bytes(build_dx_data())
    assert data.get_ascii_sentences() == ''
    assert len(data.get_port_sidescan()) == 0
    assert data.get_all_bathymetry_xyz().shape == (0, 3)
    assert data.get_all_bathymetry_xyziq(-20.0, -20.0).shape == (0, 5)
    assert data.get_port_sidescan3d().shape == (0, 4)
    assert data.get_all_sidescan3d_xyzis(-20.0, -20.0).shape == (0, 5)
