"""High-resolution binning and GeoTIFF export."""
from __future__ import annotations

import os
import time

import numpy as np
import pandas as pd

from .paths import EPSG_UTM33N, GEOTIFF

MAX_CELLS_PER_AXIS = 500_000
MAX_TOTAL_CELLS = 400_000_000


def load_xyz_en(path: str) -> np.ndarray:
    df = pd.read_csv(
        path,
        sep=r"\s+",
        comment="#",
        header=None,
        usecols=[0, 1, 2],
        names=["e", "n", "z"],
        dtype=np.float64,
    )
    return df.to_numpy()


def grid_shape(e, n, res):
    min_e, max_e = e.min(), e.max()
    min_n, max_n = n.min(), n.max()
    ncols = int(np.ceil((max_e - min_e) / res)) + 1
    nrows = int(np.ceil((max_n - min_n) / res)) + 1
    if nrows > MAX_CELLS_PER_AXIS or ncols > MAX_CELLS_PER_AXIS:
        raise ValueError(f"Grid too big: {nrows} x {ncols}")
    if nrows * ncols > MAX_TOTAL_CELLS:
        raise ValueError(f"Grid too big: {nrows * ncols:,} cells")
    return min_e, max_n, nrows, ncols


def _assign_cells(e, n, min_e, max_n, res, nrows, ncols):
    col = np.floor((e - min_e) / res).astype(np.int64)
    row = np.floor((max_n - n) / res).astype(np.int64)
    np.clip(col, 0, ncols - 1, out=col)
    np.clip(row, 0, nrows - 1, out=row)
    flat = row * ncols + col
    return flat, nrows, ncols


def bin_stats(points: np.ndarray, res: float, stats: tuple[str, ...]):
    """Bin soundings once; compute multiple per-cell statistics.

    Supported stats: median, shoal_p98, count, std
    """
    e, n, z = points[:, 0], points[:, 1], points[:, 2]
    min_e, max_n, nrows, ncols = grid_shape(e, n, res)
    flat, nrows, ncols = _assign_cells(e, n, min_e, max_n, res, nrows, ncols)
    ncell = nrows * ncols

    print(f"Binning {len(z):,} soundings -> {nrows} x {ncols} @ {res} m "
          f"({', '.join(stats)})...")
    t0 = time.time()

    count = np.bincount(flat, minlength=ncell).astype(np.int64)
    out: dict[str, np.ndarray] = {"count": count.reshape(nrows, ncols)}

    need_blocks = any(s in stats for s in ("median", "shoal_p98", "std"))
    if need_blocks:
        order = np.argsort(flat, kind="stable")
        flat_s = flat[order]
        z_s = z[order]
        bounds = np.flatnonzero(np.r_[True, flat_s[1:] != flat_s[:-1]])
        starts = bounds
        ends = np.r_[bounds[1:], len(flat_s)]

        median_flat = np.full(ncell, np.nan)
        shoal_flat = np.full(ncell, np.nan)
        std_flat = np.full(ncell, np.nan)

        for s, en in zip(starts, ends):
            block = z_s[s:en]
            cell = flat_s[s]
            if "median" in stats:
                median_flat[cell] = np.median(block)
            if "shoal_p98" in stats:
                shoal_flat[cell] = np.percentile(block, 98.0)
            if "std" in stats:
                std_flat[cell] = block.std()

        if "median" in stats:
            out["median"] = median_flat.reshape(nrows, ncols)
        if "shoal_p98" in stats:
            out["shoal_p98"] = shoal_flat.reshape(nrows, ncols)
        if "std" in stats:
            out["std"] = std_flat.reshape(nrows, ncols)

    print(f"  binning finished in {time.time() - t0:.1f}s")
    meta = {"min_e": min_e, "max_n": max_n, "nrows": nrows, "ncols": ncols, "res": res}
    return out, meta


def write_geotiff(path: str, grid: np.ndarray, min_e: float, max_n: float,
                  res: float, epsg: int = EPSG_UTM33N) -> None:
    from osgeo import gdal, osr

    rows, cols = grid.shape
    drv = gdal.GetDriverByName("GTiff")
    ds = drv.Create(
        path, cols, rows, 1, gdal.GDT_Float32,
        options=["COMPRESS=DEFLATE", "TILED=YES", "BIGTIFF=IF_SAFER"],
    )
    ds.SetGeoTransform([min_e, res, 0.0, max_n, 0.0, -res])
    srs = osr.SpatialReference()
    srs.ImportFromEPSG(int(epsg))
    ds.SetProjection(srs.ExportToWkt())
    band = ds.GetRasterBand(1)
    band.SetNoDataValue(float("nan"))
    band.WriteArray(grid.astype(np.float32))
    band.FlushCache()
    ds = None


def cell_centers(meta: dict) -> tuple[np.ndarray, np.ndarray]:
    ridx, cidx = np.indices((meta["nrows"], meta["ncols"]))
    res = meta["res"]
    x = meta["min_e"] + (cidx + 0.5) * res
    y = meta["max_n"] - (ridx + 0.5) * res
    return x, y


def export_grids(
    points_path: str,
    res: float = 0.05,
    out_dir: str = GEOTIFF,
    verbose: bool = True,
) -> tuple[dict[str, np.ndarray], dict]:
    """Build median + shoal_p98 grids and write GeoTIFFs."""
    os.makedirs(out_dir, exist_ok=True)
    pts = load_xyz_en(points_path)
    grids, meta = bin_stats(pts, res, ("median", "shoal_p98", "count"))

    tag = f"{int(res * 100):02d}cm" if res < 1 else f"{res:g}m"
    median_path = os.path.join(out_dir, f"median_{tag}.tif")
    shoal_path = os.path.join(out_dir, f"shoal_p98_{tag}.tif")
    write_geotiff(median_path, grids["median"], meta["min_e"], meta["max_n"], res)
    write_geotiff(shoal_path, grids["shoal_p98"], meta["min_e"], meta["max_n"], res)
    if verbose:
        print("Wrote", median_path)
        print("Wrote", shoal_path)
    return grids, meta
