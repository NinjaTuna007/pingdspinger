#!/usr/bin/env python3
"""Build orebro-sidescan-viewer.zip (portable viewer + *_sidescan.npz + README).

Default output:
  /media/shekharu/DATA/OneDrive/SMaRC PhD/Datasets/orebro/orebro-sidescan-viewer.zip
"""

from __future__ import annotations

import argparse
import shutil
import sys
import zipfile
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_DEFAULT_DATA = Path(
    "/media/shekharu/DATA/OneDrive/SMaRC PhD/Datasets/orebro/sidescan_fullres"
)
_DEFAULT_OUT = Path(
    "/media/shekharu/DATA/OneDrive/SMaRC PhD/Datasets/orebro/orebro-sidescan-viewer.zip"
)
_ROOT_NAME = "orebro-sidescan-viewer"

_SHARE_README = """# Örebro sidescan export viewer

Offline viewer for full-resolution sidescan waterfalls from the Örebro
(July 2026) PingDSP 3DSS survey bags. No ROS required.

## Quick start

```bash
# 1. Unzip
unzip orebro-sidescan-viewer.zip
cd orebro-sidescan-viewer

# 2. Python 3.10+ with pip
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\\Scripts\\activate
pip install -r requirements.txt

# Linux: tkinter is often a separate package
#   Ubuntu/Debian: sudo apt install python3-tk
#   Fedora:        sudo dnf install python3-tkinter

# 3. Run
python3 sidescan_export_viewer.py
# or: ./run_viewer.sh
```

Then **Open .npz…** and pick a file under `data/`.

First open of each file writes a cache (`*.log.npy` + `*.meta.npz`) next to the
npz so reloads are much faster. Cache is optional and can be deleted.

## What’s in the box

| Path | Purpose |
| --- | --- |
| `sidescan_export_viewer.py` | Tk viewer (wheel zoom, drag pan, viz knobs) |
| `pingdsp_driver/sidescan_image.py` | Shared render maths (same as live GUI) |
| `config/sidescan_default.json` | Default knobs (bronze colormap, etc.) |
| `data/*_sidescan.npz` | One full-res export per Örebro rosbag2 |
| `requirements.txt` | `numpy`, `opencv-python-headless`, `Pillow` |

JPEG previews are **not** included (npz is the source of truth). Re-render any
view from the viewer’s **Save view…**.

## Controls (match live Control GUI → Sidescan tab)

| Control | Default | Notes |
| --- | --- | --- |
| Speed compensate | **off** | Off = one row per ping (full survey, no drop). On = resample along-track so ~1 px ≈ range resolution. |
| Track stretch | 1.0 | Only with speed-comp: denser / sparser along-track. |
| Log min / max | 11.5 / 15.0 | `log1p(amp)` → black / white. Raise min or lower max for contrast. |
| Gamma | 1.0 | Mid-tones. |
| Nadir bins | 0 | Blank centre band. |
| Flatten | 0.7 | Across-track gain flatten. |
| CLAHE clip | 0.5 | Local contrast (`0` = off). |
| Despeckle | 0 | Median (`0` = off; odd ≥ 3). |
| Colormap | bronze | `copper` / `bronze` / `gray`. |

Render order: nadir → flatten → log window → CLAHE → despeckle → colormap.

**Navigation:** mouse wheel = zoom · drag = pan · **Fit** / double-click =
fit width · **100%** = 1:1 · `+`/`-` keys zoom.

Status text shows `view H×W` — with speed-comp off this equals the ping×bin
count in the file.

## NPZ schema

Each `data/<bag>_sidescan.npz` holds:

- `log` — `float16`, shape `(n_pings, n_bins)`, `log1p(|amplitude|)` for
  combined port|starboard at full sample width (~7680 or ~5696 bins).
- `along_m` — along-track metres from vehicle speed integration.
- `range_res_m` — metres per across-track bin (~0.0165 m).

Bags covered (15): `rosbag2_2026_07_07-*` through `rosbag2_2026_07_10-*`
matching the Örebro recording set.

## Memory / performance notes

- Large surveys (~30k × 7680) need a few GB of RAM when first materializing
  float32 for display; subsequent toggles reuse that buffer.
- Prefer leaving **Speed compensate** off until you need isotropic geometry.
- Cache files can be several hundred MB each; keep them on a fast disk.

## Source

Built from the `pingdspinger` package (`gui/sidescan_export_viewer.py`,
`pingdsp_driver/sidescan_image.py`). Export script:
`scripts/export_orebro_sidescan_images.py`. Docs:
`docs/SIDESCAN_EXPORT_VIEWER.md`.
"""

_REQUIREMENTS = """numpy>=1.24
opencv-python-headless>=4.8
Pillow>=10.0
"""

_RUN_SH = """#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
exec python3 sidescan_export_viewer.py "$@"
"""


def _add_file(zf: zipfile.ZipFile, src: Path, arc: str, compress: int) -> None:
    print(f"  + {arc}  ({src.stat().st_size / 1e6:.1f} MB)" if src.stat().st_size > 1e6
          else f"  + {arc}")
    zf.write(src, arcname=arc, compress_type=compress)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data", type=Path, default=_DEFAULT_DATA)
    ap.add_argument("--out", type=Path, default=_DEFAULT_OUT)
    ap.add_argument("--include-jpg", action="store_true",
                    help="Also pack *_sidescan_full.jpg previews (+~1 GB)")
    args = ap.parse_args()

    data = args.data
    if not data.is_dir():
        print(f"data dir missing: {data}", file=sys.stderr)
        return 1
    npzs = sorted(data.glob("*_sidescan.npz"))
    if not npzs:
        print(f"no *_sidescan.npz in {data}", file=sys.stderr)
        return 1

    viewer = _REPO / "gui" / "sidescan_export_viewer.py"
    ssi = _REPO / "pingdsp_driver" / "pingdsp_driver" / "sidescan_image.py"
    init = _REPO / "pingdsp_driver" / "pingdsp_driver" / "__init__.py"
    cfg = _REPO / "gui" / "sidescan" / "config" / "sidescan_default.json"
    for p in (viewer, ssi, init, cfg):
        if not p.is_file():
            print(f"missing: {p}", file=sys.stderr)
            return 1

    args.out.parent.mkdir(parents=True, exist_ok=True)
    tmp = args.out.with_suffix(".zip.partial")
    if tmp.exists():
        tmp.unlink()

    print(f"Writing {args.out} ({len(npzs)} npz)…")
    with zipfile.ZipFile(tmp, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        root = _ROOT_NAME
        # README / requirements / launcher written from strings
        for name, text in (
            ("README.md", _SHARE_README),
            ("requirements.txt", _REQUIREMENTS),
            ("run_viewer.sh", _RUN_SH),
        ):
            info = zipfile.ZipInfo(f"{root}/{name}")
            info.external_attr = (0o755 if name.endswith(".sh") else 0o644) << 16
            zf.writestr(info, text, compress_type=zipfile.ZIP_DEFLATED)
            print(f"  + {root}/{name}")

        _add_file(zf, viewer, f"{root}/sidescan_export_viewer.py", zipfile.ZIP_DEFLATED)
        _add_file(zf, init, f"{root}/pingdsp_driver/__init__.py", zipfile.ZIP_DEFLATED)
        _add_file(zf, ssi, f"{root}/pingdsp_driver/sidescan_image.py", zipfile.ZIP_DEFLATED)
        _add_file(zf, cfg, f"{root}/config/sidescan_default.json", zipfile.ZIP_DEFLATED)

        for npz in npzs:
            # Already compressed float16 — store without re-deflate.
            _add_file(zf, npz, f"{root}/data/{npz.name}", zipfile.ZIP_STORED)

        if args.include_jpg:
            for jpg in sorted(data.glob("*_sidescan_full.jpg")):
                _add_file(zf, jpg, f"{root}/data/{jpg.name}", zipfile.ZIP_STORED)

    shutil.move(str(tmp), str(args.out))
    mb = args.out.stat().st_size / (1024 ** 2)
    print(f"Done: {args.out}  ({mb:.0f} MiB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
