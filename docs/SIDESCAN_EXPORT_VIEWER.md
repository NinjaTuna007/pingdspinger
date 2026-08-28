# Offline sidescan export viewer (Örebro)

Standalone Tk viewer for full-resolution sidescan waterfalls exported from
Örebro rosbag2 recordings. Same viz knobs as the live Control GUI **Sidescan**
tab (`gui/sonar_control_gui.py::ss_render_bgr`).

## Share pack

A zip for collaborators (viewer + `*_sidescan.npz` + README) is built next to
the exports:

`…/Datasets/orebro/orebro-sidescan-viewer.zip`

Rebuild with:

```bash
python3 scripts/pack_sidescan_share.py
```

## In-repo usage

```bash
# deps: numpy, opencv-python, Pillow, tkinter (usually via python3-tk)
export PYTHONPATH=$PWD/pingdsp_driver:$PWD/gui
python3 gui/sidescan_export_viewer.py
```

Open any `*_sidescan.npz` under
`…/orebro/sidescan_fullres/` (or the pack’s `data/`). First open writes a
one-time mmap cache (`*.log.npy` + `*.meta.npz`) beside the npz for faster
reloads.

## Knobs (match live GUI)

| Control | Default | Meaning |
| --- | --- | --- |
| Speed compensate | off | Resample along-track so 1 px ≈ range resolution (optional). |
| Track stretch | 1.0 | Scale along-track density when speed-comp is on. |
| Log min / max | 11.5 / 15.0 | `log1p(amp)` window → black / white. |
| Gamma | 1.0 | Mid-tone curve. |
| Nadir bins | 0 | Blank this many bins each side of nadir. |
| Flatten | 0.7 | Across-track gain flatten (0=off…1=full). |
| CLAHE clip | 0.5 | Local contrast (0=off). |
| Despeckle | 0 | Median kernel (odd ≥3; 0=off). |
| Colormap | bronze | `copper` / `bronze` / `gray`. |

Pipeline order: nadir mask → flatten → log window → CLAHE → despeckle → colormap.

Navigation: wheel zoom, drag pan, Fit / 100% / ±, double-click fit.

## NPZ contents

| Array | Notes |
| --- | --- |
| `log` | `float16` `(n_pings, n_bins)` = `log1p(|amp|)` combined port\|starboard. |
| `along_m` | Along-track metres from `/pingdsp/speed` integration. |
| `range_res_m` | Across-track metres per bin (≈0.0165). |
| (also) | stamps / speeds may be present depending on export revision. |

Typical width is ~7680 bins (3840/side) or ~5696. Row count = ping count
(up to ~30k). With **Speed compensate** off, the view keeps every ping (1:1).

## Re-export from bags

```bash
python3 scripts/export_orebro_sidescan_images.py
```

See also `docs/SIDESCAN_VIEWER.md` for the live ROS GUI / node path.
