#!/usr/bin/env python3
"""Combine raw survey .xyz files with no cleaning -> cleaned/combined_raw.xyz.

Uses the same file list as combined_clean.xyz (see pointcloud/paths.py DEFAULT_RAW_FILES).

Usage:
    python3 combine_raw_pointclouds.py
    python3 combine_raw_pointclouds.py --files sonar_bathymetry_20260708_104912.xyz ...
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pointcloud.clean import combine_raw
from pointcloud.paths import COMBINED_RAW, DEFAULT_RAW_FILES, RAW, ensure_dirs


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--files", nargs="+", default=None,
                    help="raw .xyz filenames in pointclouds/")
    ap.add_argument("--out", default=COMBINED_RAW)
    args = ap.parse_args()

    ensure_dirs()
    files = args.files or DEFAULT_RAW_FILES
    print("Combining (no cleaning):", ", ".join(files))
    combine_raw(files, args.out, raw_dir=RAW)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
