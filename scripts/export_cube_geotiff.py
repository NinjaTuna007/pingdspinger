#!/usr/bin/env python3
"""Export 0.5 m CUBE-gridded bathymetry (.xyz + GeoTIFF) using bathycube.

Uses NOAA's CUBE algorithm (Combined Uncertainty and Bathymetry Estimator):
https://github.com/noaa-ocs-hydrography/bathycube

Outputs:
  pointclouds/cleaned/combined_cube_05m.xyz
  pointclouds/geotiff/depth_cube_05m.tif
  pointclouds/geotiff/depth_cube_05m.qml   (QGIS style, depth 0–30 m)

GeoTIFF pixel values: depth in metres, positive downward, NoData=-9999, EPSG:32633.
In QGIS set pseudocolor min=0 max=30 (or load the .qml style).

Usage:
    python3 export_cube_geotiff.py
    python3 export_cube_geotiff.py --input ../pointclouds/cleaned/combined_clean.xyz
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pointcloud.grid import export_cube
from pointcloud.paths import (
    COMBINED_CLEAN,
    COMBINED_CUBE_05M,
    DEPTH_CUBE_05M_TIF,
    GEOTIFF,
    ensure_dirs,
)


def write_qgis_style(qml_path: str) -> None:
    qml = """<!DOCTYPE qgis PUBLIC 'http://mrcc.com/qgis/qgis.dtd' 'SYSTEM'>
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
    <hillshade enabled="0"/>
  </pipe>
  <blendMode>0</blendMode>
</qgis>
"""
    with open(qml_path, "w") as fh:
        fh.write(qml)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", default=COMBINED_CLEAN)
    ap.add_argument("--resolution", type=float, default=0.5)
    ap.add_argument("--out-xyz", default=COMBINED_CUBE_05M)
    ap.add_argument("--out-tif", default=DEPTH_CUBE_05M_TIF)
    ap.add_argument("--thu", type=float, default=0.5, help="horizontal uncertainty [m]")
    ap.add_argument("--tvu", type=float, default=0.3, help="vertical uncertainty [m]")
    args = ap.parse_args()

    if not os.path.isfile(args.input):
        sys.exit(f"Input not found: {args.input}")

    ensure_dirs()
    print(f"CUBE export @ {args.resolution} m from:\n  {args.input}\n")

    export_cube(
        args.input,
        res=args.resolution,
        out_xyz=args.out_xyz,
        out_tif=args.out_tif,
        thu=args.thu,
        tvu=args.tvu,
    )

    qml = os.path.join(GEOTIFF, "depth_cube_05m.qml")
    write_qgis_style(qml)
    print("Wrote", qml)
    print("\nDone. QGIS: load", args.out_tif, "with style", qml)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
