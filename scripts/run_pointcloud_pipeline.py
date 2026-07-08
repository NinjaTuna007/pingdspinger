#!/usr/bin/env python3
"""Run the full PingDSP point-cloud pipeline.

Layout (under pingdspinger/):
    pointclouds/              raw survey .xyz files only
    pointclouds/cleaned/      cleaned combined cloud + object candidates
    pointclouds/geotiff/      gridded GeoTIFF products

Steps:
    1. Clean + combine raw surveys  -> cleaned/combined_clean.xyz
    2. Grid at high resolution      -> geotiff/depth_median_05cm.tif, depth_shoal_p98_05cm.tif
    3. Object detection             -> geotiff/object_relief_05cm.tif
                                       cleaned/object_candidates.xyz

Depth GeoTIFF convention: water depth in metres, positive downward, NoData=-9999.
For export-only (skip clean):  python3 export_bathymetry_geotiff.py

Usage:
    python3 run_pointcloud_pipeline.py
    python3 run_pointcloud_pipeline.py --resolution 0.05 --relief-threshold 0.10
    python3 run_pointcloud_pipeline.py --files sonar_bathymetry_20260708_104912.xyz ...
    python3 run_pointcloud_pipeline.py --skip-grid   # clean only
"""
from __future__ import annotations

import argparse
import os
import sys

# Allow running as scripts/run_pointcloud_pipeline.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pointcloud.clean import combine_and_clean
from pointcloud.detect import detect_objects
from pointcloud.grid import export_grids
from pointcloud.paths import (
    CLEANED,
    COMBINED_CLEAN,
    DEFAULT_RAW_FILES,
    GEOTIFF,
    OBJECT_CANDIDATES,
    RAW,
    ensure_dirs,
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--files", nargs="+", default=None,
                    help="raw .xyz filenames in pointclouds/ (default: 4 survey files)")
    ap.add_argument("--resolution", type=float, default=0.05,
                    help="grid cell size in metres (default 0.05 = 5 cm)")
    ap.add_argument("--relief-threshold", type=float, default=0.10,
                    help="min shoal_p98 - median height to flag object [m]")
    ap.add_argument("--min-support", type=int, default=3,
                    help="min soundings per cell for object detection")
    ap.add_argument("--skip-clean", action="store_true")
    ap.add_argument("--skip-grid", action="store_true")
    ap.add_argument("--skip-detect", action="store_true")
    args = ap.parse_args()

    ensure_dirs()
    files = args.files or DEFAULT_RAW_FILES
    tag = f"{int(args.resolution * 100):02d}cm" if args.resolution < 1 else f"{args.resolution:g}m"

    if not args.skip_clean:
        print("=== Step 1: clean + combine ===")
        combine_and_clean(files, COMBINED_CLEAN, raw_dir=RAW)
    elif not os.path.isfile(COMBINED_CLEAN):
        sys.exit(f"Missing {COMBINED_CLEAN}; run without --skip-clean first")

    if args.skip_grid:
        return 0

    print("\n=== Step 2: grid ===")
    grids, meta = export_grids(COMBINED_CLEAN, res=args.resolution, out_dir=GEOTIFF)

    if args.skip_detect:
        return 0

    print("\n=== Step 3: object detection ===")
    detect_objects(
        grids,
        meta,
        relief_threshold=args.relief_threshold,
        min_support=args.min_support,
        out_relief_tif=os.path.join(GEOTIFF, f"object_relief_{tag}.tif"),
        out_candidates_xyz=OBJECT_CANDIDATES,
    )
    print("\nDone.")
    print(f"  cleaned cloud : {COMBINED_CLEAN}")
    print(f"  GeoTIFFs      : {GEOTIFF}/")
    print(f"  candidates    : {OBJECT_CANDIDATES}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
