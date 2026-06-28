"""Pure-logic tests for pingdsp_driver.sidescan_image (no ROS)."""

import numpy as np

from pingdsp_driver import sidescan_image as ssi


def test_combine_ping_orientation():
    port = [1.0, 2.0, 3.0]      # nadir-first
    stbd = [4.0, 5.0, 6.0]
    row = ssi.combine_ping(port, stbd)
    # Port is flipped (outward..nadir) then starboard (nadir..outward).
    assert list(row) == [3.0, 2.0, 1.0, 4.0, 5.0, 6.0]


def test_combine_ping_one_side_only():
    assert list(ssi.combine_ping([1.0, 2.0], [])) == [2.0, 1.0]
    assert list(ssi.combine_ping([], [3.0, 4.0])) == [3.0, 4.0]
    assert ssi.combine_ping([], []).size == 0


def test_downsample_row_bins():
    row = np.arange(10, dtype=np.float32)
    out = ssi.downsample_row(row, 5)
    assert out.size == 5
    # factor=2 -> pairwise means.
    assert list(out) == [0.5, 2.5, 4.5, 6.5, 8.5]


def test_downsample_row_noop_when_short():
    row = np.arange(4, dtype=np.float32)
    out = ssi.downsample_row(row, 100)
    assert list(out) == [0, 1, 2, 3]


def test_normalize_ping_range_and_flat():
    out = ssi.normalize_ping([0.0, 1.0, 4.0, 9.0])
    assert out.dtype == np.uint8
    assert out.min() == 0
    # eps in the normaliser caps the top just below full-scale.
    assert out.max() >= 253
    assert out[1] > out[0]
    # All-zero row -> mid grey, stays visible rather than collapsing to black.
    flat = ssi.normalize_ping([0.0, 0.0, 0.0])
    assert np.all(flat == 128)


def test_normalize_ping_scrubs_nan_inf():
    out = ssi.normalize_ping([np.nan, np.inf, -np.inf, 1.0])
    assert np.all(np.isfinite(out))


def test_bronze_colormap_shape_and_endpoints():
    lut = ssi.build_bronze_lut()
    assert lut.shape == (256, 3)
    assert lut.dtype == np.uint8
    color = ssi.apply_bronze_colormap(np.array([0, 255], dtype=np.uint8))
    assert color.shape == (2, 3)


def test_ping_to_color_row():
    row = ssi.ping_to_color_row([1.0, 2.0, 3.0, 4.0], [1.0, 2.0, 3.0, 4.0],
                                target_width=4)
    assert row.ndim == 2
    assert row.shape[1] == 3       # BGR
    assert row.dtype == np.uint8


def test_build_waterfall_stacks_rows():
    rows = [ssi.ping_to_color_row([1.0, 2.0], [3.0, 4.0], target_width=4)
            for _ in range(5)]
    waterfall = ssi.build_waterfall(rows)
    assert waterfall.shape[0] == 5
    assert waterfall.shape[2] == 3


def test_build_waterfall_empty():
    assert ssi.build_waterfall([]) is None


def test_resample_row_bins_to_fixed_width():
    row = np.arange(10, dtype=np.float32)
    # Downsample by area-averaging to a fixed width.
    down = ssi.resample_row(row, 5)
    assert down.size == 5
    # Mean of all bins equals the row mean (binning conserves the average).
    assert abs(float(down.mean()) - float(row.mean())) < 1e-3
    # A constant row stays constant through binning.
    flat = ssi.resample_row(np.full(1000, 7.0, dtype=np.float32), 64)
    assert flat.size == 64
    assert np.allclose(flat, 7.0, atol=1e-3)
    # Upsampling still returns the requested width.
    up = ssi.resample_row(np.array([0.0, 9.0], dtype=np.float32), 10)
    assert up.size == 10


def test_build_waterfall_fixed_height():
    rows = [ssi.ping_to_color_row([1.0, 2.0], [3.0, 4.0], target_width=4)
            for _ in range(3)]
    img = ssi.build_waterfall(rows, height=8)
    assert img.shape[0] == 8          # padded to fixed height
    # The 3 real rows are on top; the rest is zero-filled.
    assert np.any(img[:3] != 0)
    assert np.all(img[3:] == 0)


def test_waterfall_processor_fixed_width_and_window():
    proc = ssi.WaterfallProcessor(width=64, log_min=0.0, log_max=12.0)
    # Differently-sized pings still render to the fixed width.
    r1 = proc.process(np.linspace(0, 50000, 50), np.linspace(0, 50000, 50))
    r2 = proc.process(np.linspace(0, 50000, 700), np.linspace(0, 50000, 700))
    assert r1.shape == (64, 3) and r2.shape == (64, 3)
    assert r1.dtype == np.uint8
    # Empty ping -> None.
    assert proc.process([], []) is None
    # set_width changes the output width.
    proc.set_width(128)
    r3 = proc.process(np.linspace(0, 50000, 200), np.linspace(0, 50000, 200))
    assert r3.shape == (128, 3)


def test_waterfall_processor_fixed_window_is_deterministic():
    # No normalisation: the same ping always maps to the same pixels,
    # regardless of history (consistent brightness across pings).
    proc = ssi.WaterfallProcessor(width=32, log_min=8.0, log_max=11.1)
    ping = (np.full(500, 30000.0), np.full(500, 30000.0))
    a = proc.process(*ping)
    proc.process(np.full(500, 65000.0), np.full(500, 65000.0))  # bright ping
    b = proc.process(*ping)  # same dim ping again
    assert np.array_equal(a, b)


def test_waterfall_processor_nadir_blanking():
    width = 100
    proc = ssi.WaterfallProcessor(width=width, log_min=0.0, log_max=12.0,
                                  nadir_bins=5, colormap='gray')
    n = 400
    # Bright near-range (nadir = first range bins) + a seabed amplitude ramp.
    port = np.linspace(100.0, 50000.0, n, dtype=np.float32)
    stbd = np.linspace(100.0, 50000.0, n, dtype=np.float32)
    port[:20] = 65000.0
    stbd[:20] = 65000.0
    row = proc.process(port, stbd)
    assert row.shape == (width, 3)
    c = width // 2
    # The blanked nadir band is black.
    assert np.all(row[c - 5:c + 5] == 0)
    # The seabed ramp still shows a real spread of values across the swath.
    swath = row[:c - 5, 0].astype(int)
    assert swath.ptp() > 30


def test_waterfall_processor_handles_nan_inf():
    proc = ssi.WaterfallProcessor(width=16)
    row = proc.process([np.nan, np.inf, -np.inf, 1.0, 2.0],
                       [3.0, 4.0, np.nan, 5.0, 6.0])
    assert row.shape == (16, 3)
    assert np.all(np.isfinite(row))


def test_stack_log_rows_fixed_height_and_nan_fill():
    rows = [np.array([1.0, 2.0, 3.0], dtype=np.float32) for _ in range(3)]
    img = ssi.stack_log_rows(rows, height=6)
    assert img.shape == (6, 3)
    assert np.all(np.isfinite(img[:3]))     # real rows on top
    assert np.all(np.isnan(img[3:]))        # unfilled rows are NaN (black)
    assert ssi.stack_log_rows([], height=6) is None


def test_log_to_gray_window_and_nan():
    log_img = np.array([[0.0, 5.0, 10.0, np.nan]], dtype=np.float32)
    gray = ssi.log_to_gray(log_img, log_min=0.0, log_max=10.0)
    assert gray.dtype == np.uint8
    assert gray[0, 0] == 0 and gray[0, 2] == 255
    assert gray[0, 1] == 127 or gray[0, 1] == 128   # midpoint
    assert gray[0, 3] == 0                            # NaN -> black


def test_flatten_across_track_removes_column_gradient():
    # Two columns offset by a constant per-column bias + small feature.
    rng = np.random.default_rng(0)
    base = rng.normal(10.0, 0.1, size=(50, 4)).astype(np.float32)
    bias = np.array([0.0, 2.0, 4.0, 6.0], dtype=np.float32)  # range gradient
    img = base + bias[None, :]
    before = img.mean(axis=0)
    flat = ssi.flatten_across_track(img, strength=1.0)
    after = flat.mean(axis=0)
    # Column means were spread ~6 apart; after flattening they collapse.
    assert before.ptp() > 5.0
    assert after.ptp() < 0.2
    # strength=0 is a no-op.
    assert np.allclose(ssi.flatten_across_track(img, 0.0), img)


def test_flatten_ignores_nan_columns():
    img = np.array([[1.0, np.nan, 3.0],
                    [2.0, np.nan, 5.0]], dtype=np.float32)
    out = ssi.flatten_across_track(img, strength=1.0)
    assert np.all(np.isnan(out[:, 1]))      # blank column stays blank
    assert np.all(np.isfinite(out[:, [0, 2]]))


def test_blank_nadir_zeros_centre_band():
    img = np.ones((4, 10, 3), dtype=np.uint8) * 200
    out = ssi.blank_nadir(img, 2)
    assert np.all(out[:, 3:5] == 0)         # centre +/-2 blanked
    assert np.all(out[:, :3] == 200)


def test_colormaps_shape_and_selection():
    for name in ('copper', 'bronze', 'gray'):
        lut = ssi.get_lut(name)
        assert lut.shape == (256, 3) and lut.dtype == np.uint8
    # Copper/gray are monotonic in total brightness (no false banding).
    copper = ssi.get_lut('copper').astype(int).sum(axis=1)
    assert np.all(np.diff(copper) >= 0)
    color = ssi.apply_colormap(np.array([0, 128, 255], dtype=np.uint8), 'copper')
    assert color.shape == (3, 3)
    # Unknown name falls back rather than raising.
    assert ssi.get_lut('nope').shape == (256, 3)
