#!/usr/bin/env python3
"""Export conventional QGIS-ready bathymetry GeoTIFF from a cleaned .xyz point cloud.

Outputs (in pointclouds/geotiff/):
  depth_median_<res>.tif   median seabed depth (positive metres, NoData=-9999)
  depth_median_<res>.qml   QGIS style — load alongside the .tif

Recommended for QGIS viewing:
    python3 export_bathymetry_geotiff.py --resolution 1.0 --median-only --fill-distance 20

The 5 cm grid looks speckled in QGIS because only ~25%% of cells have data; use 1 m
with gap-fill for a continuous bathymetry surface.

Usage:
    python3 export_bathymetry_geotiff.py --resolution 1.0 --median-only --fill-distance 20
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pointcloud.detect import detect_objects
from pointcloud.grid import export_grids
from pointcloud.paths import COMBINED_CLEAN, GEOTIFF, OBJECT_CANDIDATES, ensure_dirs


def write_qgis_style(qml_path: str, layer_name: str) -> None:
    """Write a blue-depth QGIS style (0–30 m) matching our depth convention."""
    qml = f"""<!DOCTYPE qgis PUBLIC 'http://mrcc.com/qgis/qgis.dtd' 'SYSTEM'>
<qgis version="3.34" styleCategories="AllStyleCategories">
  <pipe>
    <provider>
      <resampling maxOversampling="2" enabled="false" zoomedOutResamplingMethod="nearestNeighbour" zoomedInResamplingMethod="nearestNeighbour"/>
    </provider>
    <rasterrenderer alphaBand="-1" classificationMin="0" classificationMax="30" band="1" opacity="1" type="singlebandpseudocolor">
      <rasterTransparency/>
      <minMaxOrigin>
        <limits>None</limits>
        <extent>WholeRaster</extent>
        <statMin>0</statMin>
        <statMax>30</statMax>
        <statMean>0</statMean>
        <statStdDev>0</statStdDev>
      </minMaxOrigin>
      <rastershader>
        <colorramp type="gradient" name="[source]">
          <Option type="Map">
            <Option type="QString" name="color1" value="#f7fbff"/>
            <Option type="QString" name="color2" value="#08306b"/>
            <Option type="QString" name="discrete" value="0"/>
            <Option type="QString" name="rampType" value="gradient"/>
          </Option>
        </colorramp>
      </rastershader>
    </rasterrenderer>
    <brightnesscontrast brightness="0" gamma="1" contrast="0"/>
    <hillshade zFactor="1" multidirectional="0" azimuth="315" scale="3" blendMode="normal" lightAngle="45" enabled="0"/>
  </pipe>
  <blendMode>0</blendMode>
</qgis>
"""
    with open(qml_path, "w") as fh:
        fh.write(qml)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--input", default=COMBINED_CLEAN)
    ap.add_argument("--resolution", type=float, default=1.0,
                    help="grid cell size in metres (default 1.0)")
    ap.add_argument("--fill-distance", type=float, default=20.0,
                    help="fill NoData gaps up to this distance [m] (default 20, 0=off)")
    ap.add_argument("--median-only", action="store_true",
                    help="export only the median depth GeoTIFF")
    ap.add_argument("--relief-threshold", type=float, default=0.10)
    ap.add_argument("--min-support", type=int, default=3)
    ap.add_argument("--skip-detect", action="store_true")
    args = ap.parse_args()

    if not os.path.isfile(args.input):
        sys.exit(f"Input not found: {args.input}")

    ensure_dirs()
    tag = f"{int(args.resolution * 100):02d}cm" if args.resolution < 1 else f"{args.resolution:g}m"

    print(f"Exporting from:\n  {args.input}\n")
    grids, meta = export_grids(
        args.input,
        res=args.resolution,
        out_dir=GEOTIFF,
        median_only=args.median_only or args.skip_detect,
        fill_distance_m=args.fill_distance,
    )

    median_tif = os.path.join(GEOTIFF, f"depth_median_{tag}.tif")
    median_qml = os.path.join(GEOTIFF, f"depth_median_{tag}.qml")
    write_qgis_style(median_qml, f"depth_median_{tag}")
    print("Wrote", median_qml)

    if not args.skip_detect and not args.median_only:
        print()
        detect_objects(
            grids, meta,
            relief_threshold=args.relief_threshold,
            min_support=args.min_support,
            out_relief_tif=os.path.join(GEOTIFF, f"object_relief_{tag}.tif"),
            out_candidates_xyz=OBJECT_CANDIDATES,
        )

    print("\nDone.")
    print(f"  Load: {median_tif}")
    print(f"  Style: right-click layer -> Properties -> Symbology -> Style -> Load Style")
    print(f"         -> pick {median_qml}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
