"""High-resolution binning and conventional bathymetry GeoTIFF export."""
from __future__ import annotations

import os
import time

import numpy as np
import pandas as pd

from .paths import EPSG_UTM33N, GEOTIFF

MAX_CELLS_PER_AXIS = 500_000
MAX_TOTAL_CELLS = 400_000_000

# Standard hydrographic / QGIS-friendly sentinel (avoid NaN NoData — QGIS handles this poorly)
NODATA = -9999.0


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


def z_grid_to_depth(grid_z: np.ndarray) -> np.ndarray:
    """Convert negative-up elevation grid to positive-down depth with NODATA."""
    out = np.full(grid_z.shape, NODATA, dtype=np.float32)
    valid = np.isfinite(grid_z)
    out[valid] = (-grid_z[valid]).astype(np.float32)
    return out


def fill_nodata(depth: np.ndarray, res: float, max_dist_m: float) -> np.ndarray:
    """Fill small holes between survey lines using GDAL (distance in metres)."""
    from osgeo import gdal

    if max_dist_m <= 0:
        return depth

    rows, cols = depth.shape
    max_px = max(1, int(round(max_dist_m / res)))
    mem = gdal.GetDriverByName("MEM")

    ds = mem.Create("", cols, rows, 1, gdal.GDT_Float32)
    band = ds.GetRasterBand(1)
    band.SetNoDataValue(NODATA)
    band.WriteArray(depth.astype(np.float32))

    mask_ds = mem.Create("", cols, rows, 1, gdal.GDT_Byte)
    mask_band = mask_ds.GetRasterBand(1)
    mask_band.WriteArray((depth != NODATA).astype(np.uint8) * 255)

    gdal.FillNodata(
        targetBand=band,
        maskBand=mask_band,
        maxSearchDist=max_px,
        smoothingIterations=1,
    )
    filled = band.ReadAsArray()
    ds = None
    mask_ds = None
    return filled.astype(np.float32)


def write_bathymetry_geotiff(
    path: str,
    grid: np.ndarray,
    min_e: float,
    max_n: float,
    res: float,
    *,
    value_mode: str = "depth",
    description: str = "",
    epsg: int = EPSG_UTM33N,
    build_overviews: bool = True,
) -> None:
    """Write a QGIS-friendly GeoTIFF.

    value_mode:
        depth      — grid holds z (negative-up elevation); export depth = -z [m]
        depth_raw  — grid already holds positive depth with NODATA set
        relief     — grid holds a positive height anomaly [m]; written as-is
    """
    from osgeo import gdal, osr

    rows, cols = grid.shape
    out = np.array(grid, dtype=np.float32, copy=True)

    if value_mode == "depth":
        valid = np.isfinite(out)
        out[~valid] = NODATA
        out[valid] = -out[valid]
    elif value_mode == "depth_raw":
        pass
    elif value_mode == "relief":
        valid = np.isfinite(out)
        out[~valid] = NODATA
    else:
        raise ValueError(f"unknown value_mode {value_mode!r}")

    drv = gdal.GetDriverByName("GTiff")
    ds = drv.Create(
        path,
        cols,
        rows,
        1,
        gdal.GDT_Float32,
        options=[
            "COMPRESS=DEFLATE",
            "TILED=YES",
            "BIGTIFF=IF_SAFER",
            "PREDICTOR=2",
        ],
    )
    # GDAL geotransform: origin = top-left corner of top-left pixel
    ds.SetGeoTransform([min_e, res, 0.0, max_n, 0.0, -res])
    srs = osr.SpatialReference()
    srs.ImportFromEPSG(int(epsg))
    ds.SetProjection(srs.ExportToWkt())
    ds.SetMetadataItem("AREA_OR_POINT", "Area")

    band = ds.GetRasterBand(1)
    band.SetNoDataValue(NODATA)
    if description:
        band.SetDescription(description)
    band.SetMetadataItem("UNIT", "metre")
    if value_mode == "depth":
        band.SetMetadataItem(
            "TIFFTAG_IMAGEDESCRIPTION",
            "Water depth below ellipsoid (metres, positive downward)",
        )
    else:
        band.SetMetadataItem(
            "TIFFTAG_IMAGEDESCRIPTION",
            "Height above local seabed (metres, positive upward)",
        )

    band.WriteArray(out)
    band.ComputeStatistics(False)
    band.FlushCache()

    if build_overviews and min(rows, cols) >= 256:
        ds.BuildOverviews("AVERAGE", [2, 4, 8, 16, 32])
    ds = None


# Back-compat alias used by detect.py
write_geotiff = write_bathymetry_geotiff


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
    *,
    median_only: bool = False,
    fill_distance_m: float = 0.0,
) -> tuple[dict[str, np.ndarray], dict]:
    """Build median (+ optional shoal) grids and write conventional depth GeoTIFFs."""
    os.makedirs(out_dir, exist_ok=True)
    pts = load_xyz_en(points_path)
    stats: tuple[str, ...] = ("median",) if median_only else ("median", "shoal_p98", "count")
    grids, meta = bin_stats(pts, res, stats)

    tag = f"{int(res * 100):02d}cm" if res < 1 else f"{res:g}m"
    median_path = os.path.join(out_dir, f"depth_median_{tag}.tif")

    depth = z_grid_to_depth(grids["median"])
    if fill_distance_m > 0:
        if verbose:
            print(f"  filling gaps up to {fill_distance_m} m...")
        depth = fill_nodata(depth, res, fill_distance_m)

    write_bathymetry_geotiff(
        median_path, depth, meta["min_e"], meta["max_n"], res,
        value_mode="depth_raw",
        description="Median seabed depth",
    )
    if verbose:
        print("Wrote", median_path)

    if not median_only:
        shoal_path = os.path.join(out_dir, f"depth_shoal_p98_{tag}.tif")
        write_bathymetry_geotiff(
            shoal_path, grids["shoal_p98"], meta["min_e"], meta["max_n"], res,
            value_mode="depth",
            description="Robust shoal depth (98th percentile)",
        )
        if verbose:
            print("Wrote", shoal_path)

    return grids, meta


# --------------------------------------------------------------------------- #
# bathycube CUBE gridding (https://github.com/noaa-ocs-hydrography/bathycube)
# --------------------------------------------------------------------------- #
DEFAULT_THU = 0.5   # 2-sigma horizontal uncertainty [m]
DEFAULT_TVU = 0.3   # 2-sigma vertical uncertainty [m]
CUBE_METHOD = "local"
CUBE_IHO = "order1a"


def grid_cube(
    points: np.ndarray,
    res: float,
    thu: float = DEFAULT_THU,
    tvu: float = DEFAULT_TVU,
    method: str = CUBE_METHOD,
    iho: str = CUBE_IHO,
) -> tuple[dict[str, np.ndarray], dict]:
    """Run bathycube CUBE; return grids in negative-up z plus metadata."""
    from bathycube.numba_cube import run_cube_gridding

    e = points[:, 0].astype(np.float64)
    n = points[:, 1].astype(np.float64)
    z = points[:, 2].astype(np.float64)
    depth = -z  # CUBE expects positive-down depth

    min_e, max_n, nrows, ncols = grid_shape(e, n, res)
    thu_arr = np.full_like(depth, thu)
    tvu_arr = np.full_like(depth, tvu)

    print(
        f"CUBE gridding {len(depth):,} soundings -> {nrows} x {ncols} @ {res} m "
        f"({method}/{iho})..."
    )
    t0 = time.time()
    depth_grid, unc_grid, ratio_grid, numhyp_grid = run_cube_gridding(
        depth, thu_arr, tvu_arr, e, n,
        ncols, nrows, min_e, max_n,
        method, iho, res, res,
    )
    print(f"  CUBE finished in {time.time() - t0:.1f}s")

    z_grid = -np.asarray(depth_grid)  # back to negative-up elevation
    meta = {"min_e": min_e, "max_n": max_n, "nrows": nrows, "ncols": ncols, "res": res}
    grids = {
        "depth": np.asarray(depth_grid, dtype=np.float64),
        "z": z_grid,
        "uncertainty": np.asarray(unc_grid, dtype=np.float64),
        "ratio": np.asarray(ratio_grid, dtype=np.float64),
        "numhyp": np.asarray(numhyp_grid, dtype=np.int32),
    }
    return grids, meta


def write_cube_xyz(
    path: str,
    grids: dict[str, np.ndarray],
    meta: dict,
) -> int:
    """Write populated CUBE cells as easting northing z uncertainty ratio numhyp."""
    z = grids["z"]
    valid = np.isfinite(z)
    x, y = cell_centers(meta)
    rows = np.column_stack([
        x[valid], y[valid], z[valid],
        grids["uncertainty"][valid],
        grids["ratio"][valid],
        grids["numhyp"][valid],
    ])
    header = (
        "# CUBE-gridded PingDSP bathymetry\n"
        "# frame: UTM zone 33N (EPSG:32633); z is negative-up elevation [m]\n"
        "# algorithm: bathycube CUBE (Combined Uncertainty and Bathymetry Estimator)\n"
        "# columns: easting northing z uncertainty ratio numhyp"
    )
    np.savetxt(
        path,
        rows,
        fmt=["%.3f", "%.3f", "%.3f", "%.3f", "%.3f", "%d"],
        header=header,
        comments="",
    )
    return int(valid.sum())


def export_cube(
    points_path: str,
    res: float = 0.5,
    *,
    out_xyz: str,
    out_tif: str,
    thu: float = DEFAULT_THU,
    tvu: float = DEFAULT_TVU,
    verbose: bool = True,
) -> tuple[dict[str, np.ndarray], dict]:
    """Run CUBE and write gridded .xyz + conventional depth GeoTIFF."""
    os.makedirs(os.path.dirname(out_xyz) or ".", exist_ok=True)
    os.makedirs(os.path.dirname(out_tif) or ".", exist_ok=True)

    pts = load_xyz_en(points_path)
    grids, meta = grid_cube(pts, res, thu=thu, tvu=tvu)

    n_valid = write_cube_xyz(out_xyz, grids, meta)
    if verbose:
        total = meta["nrows"] * meta["ncols"]
        print(f"Wrote {out_xyz}  ({n_valid:,} / {total:,} cells, "
              f"{100 * n_valid / total:.1f}%)")

    depth = grids["depth"].astype(np.float32)
    depth[~np.isfinite(depth)] = NODATA
    write_bathymetry_geotiff(
        out_tif, depth, meta["min_e"], meta["max_n"], res,
        value_mode="depth_raw",
        description="CUBE gridded depth",
    )
    if verbose:
        print("Wrote", out_tif)

    return grids, meta
