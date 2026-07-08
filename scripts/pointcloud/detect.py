"""Object detection from median + robust shoal grids."""
from __future__ import annotations

import os

import numpy as np

from .paths import EPSG_UTM33N, GEOTIFF, OBJECT_CANDIDATES
from .grid import cell_centers, write_geotiff


def detect_objects(
    grids: dict[str, np.ndarray],
    meta: dict,
    *,
    relief_threshold: float = 0.10,
    min_support: int = 3,
    out_relief_tif: str | None = None,
    out_candidates_xyz: str | None = None,
    verbose: bool = True,
) -> np.ndarray:
    """Flag cells where robust shoal sits above median seabed with enough support.

    Returns Nx5 array: easting, northing, z_shoal, relief_m, count
    """
    median = grids["median"]
    shoal = grids["shoal_p98"]
    count = grids["count"]

    relief = shoal - median
    valid = np.isfinite(relief) & np.isfinite(shoal) & (count >= min_support)
    objects = valid & (relief >= relief_threshold)

    if verbose:
        n_obj = int(objects.sum())
        print(
            f"Object detection: relief >= {relief_threshold} m, "
            f"support >= {min_support} -> {n_obj:,} cells"
        )

    if out_relief_tif:
        os.makedirs(os.path.dirname(out_relief_tif) or ".", exist_ok=True)
        write_geotiff(out_relief_tif, relief, meta["min_e"], meta["max_n"], meta["res"],
                      EPSG_UTM33N)
        if verbose:
            print("Wrote", out_relief_tif)

    x, y = cell_centers(meta)
    if not objects.any():
        candidates = np.empty((0, 5))
    else:
        candidates = np.column_stack([
            x[objects],
            y[objects],
            shoal[objects],
            relief[objects],
            count[objects].astype(np.float64),
        ])

    if out_candidates_xyz:
        os.makedirs(os.path.dirname(out_candidates_xyz) or ".", exist_ok=True)
        header = (
            "# PingDSP object candidates (robust shoal minus median)\n"
            "# frame: UTM zone 33N (EPSG:32633); z is negative-up elevation [m]\n"
            f"# threshold: relief >= {relief_threshold} m, support >= {min_support}\n"
            "# columns: easting northing z relief_m count"
        )
        if len(candidates):
            np.savetxt(
                out_candidates_xyz,
                candidates,
                fmt=["%.3f", "%.3f", "%.3f", "%.3f", "%.0f"],
                header=header,
                comments="",
            )
        else:
            with open(out_candidates_xyz, "w") as fh:
                fh.write(header + "\n")
        if verbose:
            print("Wrote", out_candidates_xyz)

    return candidates
