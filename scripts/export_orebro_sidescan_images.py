#!/usr/bin/env python3
"""Export full-res sidescan per Örebro rosbag2: .npz (for viewer) + speed-comp JPEG.

``.npz`` holds log1p waterfall, along-track metres (from /pingdsp/speed), stamps,
and range resolution. JPEG preview applies isotropic speed compensation
(1 px along-track ≈ range_resolution metres).

Viewer: ``python3 gui/sidescan_export_viewer.py``
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / "pingdsp_driver"))
sys.path.insert(0, str(_REPO / "gui"))

from pingdsp_driver import sidescan_image as ssi  # noqa: E402
from sidescan_export_viewer import (  # noqa: E402
    integrate_along_track,
    speed_resample,
)

LOG_MIN, LOG_MAX, GAMMA = 11.5, 15.0, 1.0
FLATTEN, CLAHE, CLAHE_GRID = 0.70, 0.5, 8
COLORMAP, NADIR_BINS = "bronze", 0


def _find_bags(root: Path) -> list[Path]:
    return sorted(p for p in root.glob("rosbag2_*") if p.is_dir())


def _load_bag(bag: Path):
    """Return log HxW, stamps, speeds_at_ping, range_res."""
    from rosbags.highlevel import AnyReader

    rows, stamps = [], []
    width = 0
    range_res = 0.0165
    speed_t, speed_v = [], []

    with AnyReader([bag]) as reader:
        pcon = [c for c in reader.connections if c.msgtype == "pingdsp_msg/msg/Ping3DSS"]
        scon = [c for c in reader.connections if c.topic.endswith("/speed")]
        if not pcon:
            return None
        for conn, t, raw in reader.messages(connections=scon):
            m = reader.deserialize(raw, conn.msgtype)
            speed_t.append(t * 1e-9)
            speed_v.append(float(m.data))
        st = np.asarray(speed_t, dtype=np.float64)
        sv = np.asarray(speed_v, dtype=np.float64)
        for conn, _t, raw in reader.messages(connections=pcon):
            msg = reader.deserialize(raw, conn.msgtype)
            port = np.asarray(msg.port_sidescan_samples, dtype=np.float32)
            stbd = np.asarray(msg.starboard_sidescan_samples, dtype=np.float32)
            combined = ssi.combine_ping(port, stbd)
            if combined.size == 0:
                continue
            combined = np.nan_to_num(combined, nan=0.0, posinf=0.0, neginf=0.0)
            if width == 0:
                width = int(combined.size)
                range_res = float(msg.port_sidescan_range_resolution) or range_res
            elif combined.size != width:
                combined = ssi.resample_row(combined, width)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            stamps.append(ts)
            rows.append(np.log1p(np.abs(combined)).astype(np.float32))

    if not rows:
        return None
    stamps_a = np.asarray(stamps, dtype=np.float64)
    if st.size >= 2:
        speeds = np.interp(stamps_a, st, sv, left=sv[0], right=sv[-1])
    else:
        # fallback: assume ~constant from ping spacing * unknown — use 1.5 m/s
        speeds = np.full(stamps_a.shape, 1.5, dtype=np.float64)
    log_img = np.stack(rows, axis=0)
    along = integrate_along_track(stamps_a, speeds)
    return log_img, stamps_a, speeds, along, range_res


def _render(log_img: np.ndarray) -> np.ndarray:
    work = log_img
    if FLATTEN > 0:
        work = ssi.flatten_across_track(work, FLATTEN)
    gray = ssi.log_to_gray(work, LOG_MIN, LOG_MAX, GAMMA)
    if CLAHE > 0:
        gray = ssi.apply_clahe(gray, CLAHE, CLAHE_GRID)
        gray = ssi.blank_nadir(gray, NADIR_BINS)
    return ssi.apply_colormap(gray, COLORMAP)


def export_bag(bag: Path, out_dir: Path) -> Path | None:
    import cv2

    t0 = time.time()
    loaded = _load_bag(bag)
    if loaded is None:
        print(f"  SKIP {bag.name}: no Ping3DSS", flush=True)
        return None
    log_img, stamps, speeds, along, range_res = loaded
    out_dir.mkdir(parents=True, exist_ok=True)
    npz = out_dir / f"{bag.name}_sidescan.npz"
    np.savez_compressed(
        npz,
        log=log_img.astype(np.float16),
        stamps=stamps,
        speeds=speeds.astype(np.float32),
        along_m=along,
        range_res_m=np.float64(range_res),
    )
    # Preview: speed-compensated isotropic. JPEG max dim ~65535 — scale if needed.
    comp = speed_resample(log_img, along, range_res, stretch=1.0)
    del log_img
    bgr = _render(comp)
    jpg = out_dir / f"{bag.name}_sidescan_full.jpg"
    h, w = bgr.shape[:2]
    max_dim = 65000
    if max(h, w) > max_dim:
        scale = max_dim / float(max(h, w))
        bgr = cv2.resize(
            bgr,
            (max(int(w * scale), 1), max(int(h * scale), 1)),
            interpolation=cv2.INTER_AREA,
        )
        h, w = bgr.shape[:2]
    ok = cv2.imwrite(str(jpg), bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
    if not ok:
        raise RuntimeError(f"cv2.imwrite failed for {jpg} ({h}x{w})")
    track = float(along[-1] - along[0]) if along.size else 0.0
    print(
        f"  OK {bag.name}: jpg {h}x{w}  track={track:.0f}m  "
        f"res={range_res*100:.1f}cm  npz={npz.stat().st_size/1e6:.1f}MB  "
        f"{time.time()-t0:.1f}s",
        flush=True,
    )
    return jpg


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--dataset",
        type=Path,
        default=Path("/media/shekharu/DATA/OneDrive/SMaRC PhD/Datasets/orebro"),
    )
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--bags", nargs="*", default=None)
    args = ap.parse_args()
    root = args.dataset
    out_dir = args.out or (root / "sidescan_fullres")
    bags = _find_bags(root)
    if args.bags:
        bags = [b for b in bags if b.name in set(args.bags)]
    if not bags:
        print("No bags", file=sys.stderr)
        return 1
    print(f"Exporting {len(bags)} → {out_dir} (npz + speed-comp JPEG)")
    t0 = time.time()
    ok = 0
    for bag in bags:
        try:
            if export_bag(bag, out_dir):
                ok += 1
        except Exception as e:  # noqa: BLE001
            print(f"  FAIL {bag.name}: {e}", flush=True)
    print(f"Done {ok}/{len(bags)} in {time.time()-t0:.1f}s")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
