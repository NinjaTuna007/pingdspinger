"""Filesystem layout for the PingDSP point-cloud pipeline."""
from __future__ import annotations

import os

PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
POINTCLOUDS = os.path.join(PKG_ROOT, "pointclouds")
RAW = POINTCLOUDS
CLEANED = os.path.join(POINTCLOUDS, "cleaned")
GEOTIFF = os.path.join(POINTCLOUDS, "geotiff")

COMBINED_CLEAN = os.path.join(CLEANED, "combined_clean.xyz")
OBJECT_CANDIDATES = os.path.join(CLEANED, "object_candidates.xyz")

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
