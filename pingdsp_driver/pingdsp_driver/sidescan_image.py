#!/usr/bin/env python3
"""
Sidescan waterfall image helpers for the PingDSP 3DSS-DX.

This module holds the *pure* image-building logic that used to live inside
``tdss_driver.py``. Keeping it free of ROS and socket dependencies means it can
be unit tested directly (see ``test/test_sidescan_image.py``) and reused by the
standalone ``sidescan_viewer_node`` without dragging in the whole driver.

There are two ways to turn pings into pixels here:

* the stateless per-ping helpers (``combine_ping`` -> ``downsample_row`` ->
  ``normalize_ping`` -> ``apply_bronze_colormap``, wrapped by
  ``ping_to_color_row``). These min/max normalise each ping on its own, which
  is simple and unit-testable but flickers row-to-row and lets the nadir
  overpower the swath; and

* :class:`WaterfallProcessor` (the live viewer's front end) bins every ping to
  a **fixed** pixel width and converts it to ``log1p(amp)``; the display side
  (fixed ``[log_min, log_max]`` window + optional across-track flatten / CLAHE /
  despeckle + colormap) is applied to the whole stacked waterfall at publish
  time via the module helpers (:func:`stack_log_rows`, :func:`flatten_across_track`,
  :func:`log_to_gray`, :func:`apply_clahe`, :func:`despeckle_gray`).

With every enhancement off this is just a constant log transfer per pixel - no
normalisation - so brightness never flickers; the optional steps only ever add
spatial corrections, never per-ping temporal scaling.
"""

import warnings

import numpy as np


# Default number of pings to keep in a waterfall. One row per ping.
DEFAULT_NUM_PINGS = 2048

# Default rendered width in pixels. Raw swaths can be many thousands of samples
# wide; binning down to this averages out per-sample speckle. At 2048 a frame is
# ~2048 x 2048 x 3 bytes; drop it if streaming gets heavy.
DEFAULT_TARGET_WIDTH = 2048


def combine_ping(port_samples, starboard_samples):
    """
    Combine port + starboard sidescan samples into a single across-track row.

    Port is flipped so it reads outward-to-nadir on the left, starboard reads
    nadir-to-outward on the right, matching a conventional waterfall layout.

    Args:
        port_samples: 1D array-like of port amplitudes (nadir-first).
        starboard_samples: 1D array-like of starboard amplitudes (nadir-first).

    Returns:
        1D float32 numpy array. Empty if both inputs are empty.
    """
    port = np.asarray(port_samples, dtype=np.float32).ravel()
    stbd = np.asarray(starboard_samples, dtype=np.float32).ravel()

    if port.size > 0 and stbd.size > 0:
        return np.concatenate([np.flip(port), stbd])
    if port.size > 0:
        return np.flip(port)
    return stbd


def downsample_row(row, target_width):
    """
    Bin-average ``row`` down to at most ``target_width`` samples.

    Rows shorter than (or equal to) the target are returned unchanged. The
    trailing samples that do not divide evenly into bins are dropped, exactly
    as the original driver did.

    Args:
        row: 1D array-like of samples.
        target_width: desired maximum width in samples (>0).

    Returns:
        1D float32 numpy array.
    """
    row = np.asarray(row, dtype=np.float32).ravel()
    original_width = row.size
    target_width = int(target_width)

    if target_width <= 0 or original_width <= target_width:
        return row

    factor = original_width // target_width
    trim = (original_width // factor) * factor
    return row[:trim].reshape(-1, factor).mean(axis=1)


def resample_row(row, width):
    """
    Resample ``row`` to exactly ``width`` samples by area-averaging (binning).

    Always returns a row of length ``width``: when downsampling (the usual case,
    raw swaths are thousands of samples) each output pixel is the *mean* of the
    raw samples falling in its bin, which both fixes the width (stable image
    size) and averages out per-sample speckle. Implemented with a cumulative
    sum so it is exact and fast for any ratio.

    Args:
        row: 1D array-like of samples.
        width: desired output width in samples (>0).

    Returns:
        1D float32 numpy array of length ``width`` (or the input unchanged when
        ``width`` <= 0 or already matches / the row is empty).
    """
    row = np.asarray(row, dtype=np.float32).ravel()
    n = row.size
    width = int(width)
    if width <= 0 or n == 0 or n == width:
        return row
    # Bin edges in input-sample coordinates; average = (cumsum at right edge -
    # cumsum at left edge) / bin span. bin span is constant (n / width).
    cs = np.concatenate(([0.0], np.cumsum(row, dtype=np.float64)))
    edges = np.linspace(0.0, n, width + 1)
    sums = np.interp(edges, np.arange(n + 1), cs)
    return (np.diff(sums) * (width / n)).astype(np.float32)


def normalize_ping(row):
    """
    Square-root compress and per-ping normalise a row to uint8 (0-255).

    NaN/inf are scrubbed to 0. A flat row (all equal) maps to mid-grey 128, so
    it stays visible rather than collapsing to black.

    Args:
        row: 1D array-like of amplitudes.

    Returns:
        1D uint8 numpy array of the same length as ``row``.
    """
    ping = np.abs(np.asarray(row, dtype=np.float32).ravel())
    ping = np.nan_to_num(ping, nan=0.0, posinf=0.0, neginf=0.0)
    ping = np.clip(ping, 0.0, None)
    ping_sqrt = np.sqrt(ping)

    eps = 1e-6
    lo, hi = ping_sqrt.min(initial=0.0), ping_sqrt.max(initial=0.0)
    if hi > lo + eps:
        norm = (ping_sqrt - lo) / (hi - lo + eps) * 255.0
        return np.clip(norm, 0.0, 255.0).astype(np.uint8)
    return np.full(ping_sqrt.shape, 128, dtype=np.uint8)


def build_bronze_lut():
    """
    Build the 256x3 (BGR) bronze colormap lookup table.

    Low returns are dark blue, ramping through bronze and yellow to white at
    the high end. This is the same mapping the driver used to publish inline.

    Returns:
        (256, 3) uint8 numpy array indexed by intensity (0-255), BGR order.
    """
    lut = np.zeros((256, 3), dtype=np.uint8)
    for i in range(256):
        if i < 64:
            t = i / 64.0
            lut[i] = [int(40 + t * 100), int(t * 30), int(t * 10)]
        elif i < 128:
            t = (i - 64) / 64.0
            lut[i] = [int(140 - t * 60), int(30 + t * 80), int(10 + t * 70)]
        elif i < 192:
            t = (i - 128) / 64.0
            lut[i] = [int(80 - t * 30), int(110 + t * 100), int(80 + t * 130)]
        else:
            t = (i - 192) / 63.0
            lut[i] = [int(50 + t * 205), int(210 + t * 45), int(210 + t * 45)]
    return lut


def build_copper_lut():
    """
    Build the 256x3 (BGR) copper colormap LUT (black -> copper -> bright).

    A clean, monotonic brightness ramp (the classic ``copper`` map): low
    returns are black, ramping smoothly through copper/bronze to near-white.
    Being monotonic in luminance it reads naturally as "more backscatter =
    brighter" without the false banding a multi-segment map can introduce.

    Returns:
        (256, 3) uint8 numpy array indexed by intensity (0-255), BGR order.
    """
    t = np.linspace(0.0, 1.0, 256)
    r = np.clip(t * 1.25, 0.0, 1.0)
    g = np.clip(t * 0.78, 0.0, 1.0)
    b = np.clip(t * 0.50, 0.0, 1.0)
    lut = np.zeros((256, 3), dtype=np.uint8)
    lut[:, 0] = (b * 255).astype(np.uint8)   # B
    lut[:, 1] = (g * 255).astype(np.uint8)   # G
    lut[:, 2] = (r * 255).astype(np.uint8)   # R
    return lut


def build_gray_lut():
    """Build a plain 256x3 (BGR) grayscale LUT."""
    ramp = np.arange(256, dtype=np.uint8)
    return np.repeat(ramp[:, None], 3, axis=1)


# Module-level LUTs so we build them once, not per ping.
_BRONZE_LUT = build_bronze_lut()
_COPPER_LUT = build_copper_lut()
_GRAY_LUT = build_gray_lut()
_LUTS = {'bronze': _BRONZE_LUT, 'copper': _COPPER_LUT, 'gray': _GRAY_LUT}

# Default colormap name for the live viewer.
DEFAULT_COLORMAP = 'copper'


def get_lut(name):
    """Return the (256,3) BGR LUT for ``name`` (falls back to copper)."""
    return _LUTS.get(str(name).lower(), _COPPER_LUT)


def apply_colormap(gray, name=DEFAULT_COLORMAP):
    """Map a uint8 grayscale array through the named colormap (BGR uint8).

    For a 2-D uint8 image this uses ``cv2.LUT`` (~30x faster than numpy fancy
    indexing on a multi-megapixel frame); it falls back to the plain numpy
    lookup for non-2-D / non-uint8 inputs or if OpenCV is unavailable. Both
    paths return the identical table lookup, just at very different speeds.
    """
    lut = get_lut(name)
    gray = np.asarray(gray)
    if gray.dtype == np.uint8 and gray.ndim == 2:
        try:
            import cv2
            src = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            return cv2.LUT(src, lut.reshape(1, 256, 3))
        except Exception:  # noqa: BLE001 - any cv2 issue -> numpy fallback
            pass
    return lut[gray]


def apply_bronze_colormap(gray):
    """
    Map a uint8 grayscale array through the bronze colormap.

    Args:
        gray: numpy array of uint8 intensities (any shape).

    Returns:
        numpy array shaped ``gray.shape + (3,)``, BGR, uint8.
    """
    gray = np.asarray(gray)
    return _BRONZE_LUT[gray]


def ping_to_color_row(port_samples, starboard_samples,
                      target_width=DEFAULT_TARGET_WIDTH):
    """
    Run the full single-ping pipeline and return a coloured BGR row.

    Args:
        port_samples: port amplitudes (nadir-first).
        starboard_samples: starboard amplitudes (nadir-first).
        target_width: max rendered width in samples.

    Returns:
        (width, 3) uint8 BGR array, or None if there were no samples.
    """
    combined = combine_ping(port_samples, starboard_samples)
    if combined.size == 0:
        return None
    combined = downsample_row(combined, target_width)
    gray = normalize_ping(combined)
    return apply_bronze_colormap(gray)


def build_waterfall(color_rows, height=None):
    """
    Stack coloured rows into a waterfall image, newest first (top).

    Rows of differing widths are right-padded with zeros to the widest row so
    the result is a clean rectangular image even if the swath width changed
    mid-survey. When ``height`` is given the image is always that many rows tall
    (older rows beyond it dropped, the bottom zero-filled when the buffer is not
    yet full) so the published image keeps a constant size and viewers do not
    re-lay-out on every frame.

    Args:
        color_rows: iterable of (width, 3) uint8 BGR rows, ordered
            newest-to-oldest.
        height: fixed output height in rows, or None to size to the input.

    Returns:
        (height, width, 3) uint8 BGR image, or None if no rows.
    """
    rows = [np.asarray(r, dtype=np.uint8) for r in color_rows if r is not None]
    rows = [r for r in rows if r.size > 0]
    if not rows:
        return None

    width = max(r.shape[0] for r in rows)
    out_h = len(rows) if height is None else int(height)
    img = np.zeros((out_h, width, 3), dtype=np.uint8)
    for i, r in enumerate(rows[:out_h]):
        img[i, :r.shape[0], :] = r
    return img


def log_to_gray(log_img, log_min, log_max, gamma=1.0):
    """
    Map a log-amplitude array through the fixed ``[log_min, log_max]`` window.

    This is the *no-normalisation* transfer function: a constant linear map of
    ``log1p(amp)`` to 0-255 (values outside the window clip), optional gamma.
    Non-finite entries (NaN = blanked nadir / unfilled rows) map to 0 (black).

    Args:
        log_img: float array of ``log1p(amp)`` values (any shape).
        log_min: log value mapped to 0 (black).
        log_max: log value mapped to 255 (white).
        gamma: <1 brightens mid-tones, >1 darkens.

    Returns:
        uint8 array of the same shape.
    """
    log_img = np.asarray(log_img, dtype=np.float32)
    span = max(float(log_max) - float(log_min), 1e-6)
    # Work in one fresh buffer with in-place ops to avoid the extra full-frame
    # temporaries (isfinite + where) the readable form allocates. np.clip maps
    # +/-inf into [0, 1] and leaves NaN as NaN; nan_to_num then sends the
    # blanked / unfilled NaNs to 0 (black).
    norm = log_img - float(log_min)
    norm *= (1.0 / span)
    np.clip(norm, 0.0, 1.0, out=norm)
    if gamma != 1.0:
        np.power(norm, float(gamma), out=norm)
    np.nan_to_num(norm, copy=False, nan=0.0)
    norm *= 255.0
    return norm.astype(np.uint8)


def flatten_across_track(log_img, strength=1.0):
    """
    Flatten the across-track (range-dependent) gain in the log domain.

    Side-scan backscatter falls off with range, so each across-track column
    sits at a different mean brightness; that gradient eats dynamic range that
    should be spent on *features*. Subtracting each column's mean (computed over
    all buffered pings) re-levels every column to the global mean, so the
    seabed background goes uniform and targets / shadows stand out. This is the
    classic empirical-gain / beam-pattern correction, done before the fixed
    window so nothing is clipped first.

    ``strength`` in [0, 1] blends between no correction (0) and full flattening
    (1). Columns that are entirely blanked (NaN, e.g. nadir) are left untouched.

    The per-column mean is taken over *every* buffered ping. (Estimating it
    from a strided row subsample is cheaper but makes the correction wobble
    frame-to-frame on columns with intermittent dropped pings - the starboard
    shimmer - so we average all rows for a stable result.)

    Args:
        log_img: (rows, width) float array of ``log1p(amp)``; NaN = blank.
        strength: correction strength, 0..1.

    Returns:
        (rows, width) float32 array, NaN preserved.
    """
    log_img = np.asarray(log_img, dtype=np.float32)
    if log_img.ndim != 2 or strength <= 0.0:
        return log_img
    # nanmean over an all-NaN (fully blanked) column is an expected NaN, not a
    # problem - silence the "Mean of empty slice" RuntimeWarning it raises.
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', category=RuntimeWarning)
        col_mean = np.nanmean(log_img, axis=0)            # (width,)
        global_mean = float(np.nanmean(col_mean))
    # Per-column offset to remove; 0 where a column has no data (NaN).
    delta = np.where(np.isfinite(col_mean),
                     (col_mean - global_mean) * float(strength),
                     0.0).astype(np.float32)
    # float32 - float32 stays float32, so skip the redundant full-frame copy
    # that a trailing .astype(float32) would make.
    return log_img - delta[None, :]


def apply_clahe(gray, clip=2.0, grid=8):
    """
    Contrast-Limited Adaptive Histogram Equalisation on a uint8 image.

    CLAHE equalises contrast in local tiles, so subtle local texture (ripples,
    faint targets) pops without a global stretch blowing out the bright areas.
    ``clip`` is the contrast limit (higher = punchier but noisier); ``grid`` is
    the tile count per axis. ``clip <= 0`` returns the input unchanged.

    Args:
        gray: (H, W) uint8 image.
        clip: CLAHE clip limit (0 disables).
        grid: tile grid size per axis.

    Returns:
        (H, W) uint8 image.
    """
    gray = np.asarray(gray, dtype=np.uint8)
    if clip <= 0.0 or gray.ndim != 2:
        return gray
    import cv2
    clahe = cv2.createCLAHE(clipLimit=float(clip),
                            tileGridSize=(int(grid), int(grid)))
    return clahe.apply(gray)


def despeckle_gray(gray, ksize=3):
    """
    Median-filter a uint8 image to suppress speckle and thin streaks.

    A small median knocks out single-pixel speckle and the odd bad-ping line
    while preserving edges. ``ksize <= 1`` (or even) returns the input.

    Args:
        gray: (H, W) uint8 image.
        ksize: odd median kernel size (>=3 to take effect).

    Returns:
        (H, W) uint8 image.
    """
    gray = np.asarray(gray, dtype=np.uint8)
    ksize = int(ksize)
    if ksize < 3 or ksize % 2 == 0 or gray.ndim != 2:
        return gray
    import cv2
    return cv2.medianBlur(gray, ksize)


def blank_nadir(img, nadir_bins):
    """Zero a band of ``nadir_bins`` columns each side of the image centre."""
    nb = int(nadir_bins)
    if nb <= 0 or img.ndim < 2:
        return img
    w = img.shape[1]
    if 2 * nb >= w:
        return img
    c = w // 2
    img[:, c - nb:c + nb] = 0
    return img


def stack_log_rows(log_rows, height):
    """
    Stack 1D log-amplitude rows (newest-first) into a fixed-height float image.

    Rows are placed newest-on-top; unfilled rows below stay NaN (rendered
    black). Mixed widths are right-padded with NaN. NaN is used (not 0) so
    unfilled / blanked pixels are excluded from across-track statistics.

    Args:
        log_rows: iterable of 1D float arrays, newest-to-oldest.
        height: fixed output height in rows.

    Returns:
        (height, width) float32 array, or None if there are no rows.
    """
    rows = [np.asarray(r, dtype=np.float32).ravel() for r in log_rows
            if r is not None and np.size(r) > 0]
    if not rows:
        return None
    width = max(r.size for r in rows)
    out_h = int(height)
    img = np.full((out_h, width), np.nan, dtype=np.float32)
    for i, r in enumerate(rows[:out_h]):
        img[i, :r.size] = r
    return img


class WaterfallProcessor:
    """
    Per-ping front end of the sidescan renderer: bin -> log.

    Each ping is binned to a fixed ``width`` (averaging denoises speckle and
    keeps the image small + stable-sized) and converted to ``log1p(amp)``. The
    near-nadir band (``nadir_bins`` pixels each side of centre) is set to NaN so
    it renders black *and* is excluded from across-track statistics.

    The *display* transfer (fixed ``[log_min, log_max]`` window, gamma, optional
    across-track flatten / CLAHE / despeckle, colormap) is applied to the whole
    stacked waterfall by the node at publish time (see :func:`log_to_gray`,
    :func:`flatten_across_track`, :func:`apply_clahe`). With every enhancement
    off the result is exactly the plain fixed-log image - no normalisation.

    ``log_min``/``log_max``/``gamma``/``colormap`` are kept here so the
    convenience :meth:`process` (single ping -> coloured row, used in tests and
    by simple callers) reproduces that same fixed mapping.
    """

    def __init__(self, width, log_min=11.5, log_max=14.5, gamma=1.0,
                 colormap=DEFAULT_COLORMAP, nadir_bins=0):
        """Create a processor rendering to ``width`` pixels per row."""
        self.width = int(width)
        self.log_min = float(log_min)
        self.log_max = float(log_max)
        self.gamma = float(gamma)
        self.colormap = colormap
        self.nadir_bins = int(nadir_bins)

    def set_width(self, width):
        """Change the render width (no internal state to reset)."""
        self.width = int(width)

    def process_row(self, port_samples, starboard_samples):
        """
        Bin one ping to ``width`` and return its ``log1p(amp)`` row (float32).

        The near-nadir band is set to NaN. Returns ``None`` for an empty ping.
        """
        combined = combine_ping(port_samples, starboard_samples)
        if combined.size == 0:
            return None
        # Scrub non-finite samples before binning so they cannot smear across
        # neighbouring bins via the cumulative sum.
        combined = np.nan_to_num(combined, nan=0.0, posinf=0.0, neginf=0.0)
        row = resample_row(combined, self.width)
        logv = np.log1p(np.abs(row)).astype(np.float32)
        nb = self.nadir_bins
        w = logv.size
        if nb > 0 and 2 * nb < w:
            c = w // 2
            logv[c - nb:c + nb] = np.nan
        return logv

    def process(self, port_samples, starboard_samples):
        """
        Render one ping to a coloured BGR row of length ``self.width``.

        Convenience for tests / simple callers: applies the fixed log window and
        colormap to a single row (no 2D enhancements). Returns ``None`` when the
        ping carries no samples.
        """
        logv = self.process_row(port_samples, starboard_samples)
        if logv is None:
            return None
        gray = log_to_gray(logv, self.log_min, self.log_max, self.gamma)
        return apply_colormap(gray, self.colormap)
