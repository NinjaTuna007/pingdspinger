"""Filesystem layout for the PingDSP point-cloud pipeline."""
from __future__ import annotations

import os

PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
POINTCLOUDS = os.path.join(PKG_ROOT, "pointclouds")
RAW = POINTCLOUDS
CLEANED = os.path.join(POINTCLOUDS, "cleaned")
GEOTIFF = os.path.join(POINTCLOUDS, "geotiff")

COMBINED_CLEAN = os.path.join(CLEANED, "combined_clean.xyz")
COMBINED_CUBE_05M = os.path.join(CLEANED, "combined_cube_05m.xyz")
OBJECT_CANDIDATES = os.path.join(CLEANED, "object_candidates.xyz")
DEPTH_CUBE_05M_TIF = os.path.join(GEOTIFF, "depth_cube_05m.tif")


def cube_xyz_path(res: float) -> str:
    tag = f"{int(res * 100):02d}cm" if res < 1 else f"{res:g}m"
    return os.path.join(CLEANED, f"cube_{tag}.xyz")


def cube_geotiff_path(res: float) -> str:
    tag = f"{int(res * 100):02d}cm" if res < 1 else f"{res:g}m"
    return os.path.join(GEOTIFF, f"depth_cube_{tag}.tif")

# Default raw surveys to combine (override with --files)
DEFAULT_RAW_FILES = [
    "sonar_bathymetry_20260708_104912.xyz",
    "sonar_bathymetry_20260708_111313.xyz",
    "sonar_bathymetry_20260708_115423.xyz",
    "sonar_bathymetry_20260708_143206.xyz",
]

EPSG_UTM33N = 32633  # Örebro, Sweden (verified from coordinates)


def ensure_dirs() -> None:
    os.makedirs(CLEANED, exist_ok=True)
    os.makedirs(GEOTIFF, exist_ok=True)
