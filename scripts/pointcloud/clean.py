"""Structure-preserving cleaning for PingDSP bathymetry point clouds."""
from __future__ import annotations

import os
import time

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

from .paths import RAW

# --- Grid / shared ----------------------------------------------------------- #
XY_CELL = 3.0
MIN_CELL_PTS = 8
SOR_K = 16
SOR_STD_RATIO = 2.0
SOR2_STD_RATIO = 2.5
SOR_WORKERS = -1

# --- Boat pass 1 ------------------------------------------------------------- #
P1_SURFACE_PCTILE = 99.5
P1_NEAR_SURFACE = 3.0
P1_MIN_ABOVE_SEABED = 4.0
P1_SEABED_PCTILE = 15.0

# --- Boat pass 2 (residual trajectory) --------------------------------------- #
P2_SURFACE_PCTILE = 98.5
P2_NEAR_SURFACE = 2.5
P2_MIN_ABOVE_SEABED = 3.0
P2_SEABED_PCTILE = 30.0

# --- Water-column spike removal (above local seabed only; no below-seabed) --- #
WATER_ABOVE_SEABED = 2.0  # [m] drop points floating this far above local seabed
WATER_SEABED_PCTILE = 25.0


def load_xyz(path: str) -> np.ndarray:
    df = pd.read_csv(
        path,
        sep=r"\s+",
        comment="#",
        header=None,
        names=["e", "n", "z", "i"],
        dtype=np.float64,
    )
    return df.to_numpy()


def write_xyz(path: str, pts: np.ndarray, header_lines: list[str]) -> None:
    np.savetxt(
        path,
        pts,
        fmt=["%.3f", "%.3f", "%.3f", "%.3f"],
        header="\n".join(header_lines),
        comments="",
    )


def _cell_indices(x: np.ndarray, y: np.ndarray, cell: float):
    gx = np.floor((x - x.min()) / cell).astype(np.int64)
    gy = np.floor((y - y.min()) / cell).astype(np.int64)
    nrows = gx.max() + 1
    ncols = gy.max() + 1
    flat = gx * ncols + gy
    return flat, nrows, ncols


def local_seabed_at_points(pts: np.ndarray, pctile: float) -> tuple[np.ndarray, np.ndarray]:
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    flat, nrows, ncols = _cell_indices(x, y, XY_CELL)
    order = np.argsort(flat, kind="stable")
    flat_s = flat[order]
    z_s = z[order]
    bounds = np.flatnonzero(np.r_[True, flat_s[1:] != flat_s[:-1]])
    starts = bounds
    ends = np.r_[bounds[1:], len(flat_s)]

    seabed = np.full(len(pts), np.nan)
    counts = np.zeros(len(pts), dtype=np.int64)
    for s, e in zip(starts, ends):
        block = z_s[s:e]
        ref = np.percentile(block, pctile)
        idx = order[s:e]
        seabed[idx] = ref
        counts[idx] = e - s
    return seabed, counts


def boat_surface_mask(
    pts: np.ndarray,
    *,
    surface_pctile: float,
    near_surface: float,
    min_above_seabed: float,
    seabed_pctile: float,
) -> np.ndarray:
    z = pts[:, 2]
    surface = np.percentile(z, surface_pctile)
    seabed, counts = local_seabed_at_points(pts, seabed_pctile)
    height_above = z - seabed
    return (
        (z > surface - near_surface)
        & (height_above > min_above_seabed)
        & (counts >= MIN_CELL_PTS)
        & np.isfinite(seabed)
    )


def water_column_mask(pts: np.ndarray) -> np.ndarray:
    """Remove points floating in the water column above the local seabed."""
    z = pts[:, 2]
    seabed, counts = local_seabed_at_points(pts, WATER_SEABED_PCTILE)
    return (
        (z > seabed + WATER_ABOVE_SEABED)
        & (counts >= MIN_CELL_PTS)
        & np.isfinite(seabed)
    )


def sor_mask(pts: np.ndarray, std_ratio: float = SOR_STD_RATIO) -> np.ndarray:
    xyz = pts[:, :3]
    tree = cKDTree(xyz)
    dist, _ = tree.query(xyz, k=SOR_K + 1, workers=SOR_WORKERS)
    mean_dist = dist[:, 1:].mean(axis=1)
    thresh = mean_dist.mean() + std_ratio * mean_dist.std()
    return mean_dist <= thresh


def _pct(before: int, after: int) -> str:
    if before == 0:
        return "0.00%"
    return f"{100 * (before - after) / before:.2f}%"


def clean_points(pts: np.ndarray, verbose: bool = True, label: str = "") -> np.ndarray:
    """Best-quality cleaning: boat x2 + SOR + water-column spikes + light SOR."""
    t0 = time.time()
    n0 = len(pts)

    boat1 = boat_surface_mask(
        pts,
        surface_pctile=P1_SURFACE_PCTILE,
        near_surface=P1_NEAR_SURFACE,
        min_above_seabed=P1_MIN_ABOVE_SEABED,
        seabed_pctile=P1_SEABED_PCTILE,
    )
    pts = pts[~boat1]
    n1 = len(pts)

    keep1 = sor_mask(pts, std_ratio=SOR_STD_RATIO)
    pts = pts[keep1]
    n2 = len(pts)

    boat2 = boat_surface_mask(
        pts,
        surface_pctile=P2_SURFACE_PCTILE,
        near_surface=P2_NEAR_SURFACE,
        min_above_seabed=P2_MIN_ABOVE_SEABED,
        seabed_pctile=P2_SEABED_PCTILE,
    )
    pts = pts[~boat2]
    n3 = len(pts)

    water = water_column_mask(pts)
    pts = pts[~water]
    n4 = len(pts)

    keep2 = sor_mask(pts, std_ratio=SOR2_STD_RATIO)
    pts = pts[keep2]
    n5 = len(pts)

    if verbose:
        print(
            f"  {label}: {n0:,} in"
            f" -> boat1 -{n0 - n1:,} ({_pct(n0, n1)})"
            f" -> sor1 -{n1 - n2:,} ({_pct(n1, n2)})"
            f" -> boat2 -{n2 - n3:,} ({_pct(n2, n3)})"
            f" -> water -{n3 - n4:,} ({_pct(n3, n4)})"
            f" -> sor2 -{n4 - n5:,} ({_pct(n4, n5)})"
            f" -> {n5:,} kept  [{time.time() - t0:.1f}s]"
        )
    return pts


def clean_file(path: str, verbose: bool = True) -> np.ndarray:
    return clean_points(load_xyz(path), verbose=verbose, label=os.path.basename(path))


def combine_raw(
    filenames: list[str],
    out_path: str,
    raw_dir: str = RAW,
    verbose: bool = True,
) -> np.ndarray:
    """Stack raw surveys with no cleaning."""
    all_pts = []
    for name in filenames:
        path = os.path.join(raw_dir, name)
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        pts = load_xyz(path)
        if verbose:
            print(f"  {name}: {len(pts):,} points")
        all_pts.append(pts)
    combined = np.vstack(all_pts)
    if verbose:
        print(f"Combined total: {len(combined):,} points (no cleaning)")
    write_xyz(
        out_path,
        combined,
        [
            "# PingDSP bathymetry point cloud (combined, raw)",
            "# frame: UTM zone 33N (EPSG:32633); load in CloudCompare and accept the global shift",
            "# source files: see DEFAULT_RAW_FILES in scripts/pointcloud/paths.py",
            "# columns: easting northing z intensity",
        ],
    )
    if verbose:
        print("Wrote", out_path)
    return combined


def combine_and_clean(
    filenames: list[str],
    out_path: str,
    raw_dir: str = RAW,
    verbose: bool = True,
) -> np.ndarray:
    all_pts = []
    for name in filenames:
        path = os.path.join(raw_dir, name)
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        all_pts.append(clean_file(path, verbose=verbose))
    combined = np.vstack(all_pts)
    if verbose:
        print(f"Combined total: {len(combined):,} points")
    write_xyz(
        out_path,
        combined,
        [
            "# PingDSP bathymetry point cloud (combined + cleaned)",
            "# frame: UTM zone 33N (EPSG:32633); load in CloudCompare and accept the global shift",
            "# cleaning: boat x2 + SOR + water-column spike removal + light SOR",
            "# columns: easting northing z intensity",
        ],
    )
    if verbose:
        print("Wrote", out_path)
    return combined
