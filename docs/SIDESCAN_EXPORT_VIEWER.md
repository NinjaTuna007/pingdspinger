# Offline sidescan export viewer

Standalone Tk viewer for full-resolution sidescan waterfalls exported from
rosbag2 recordings to `*_sidescan.npz`. Same viz knobs as the live Control GUI
**Sidescan** tab (`gui/sonar_control_gui.py::ss_render_bgr`). It has no ROS
dependency, so it can be zipped up with a set of `.npz` files and handed to
someone without a ROS install.

## Usage

```bash
# deps: numpy, opencv-python, Pillow, tkinter (usually via python3-tk)
export PYTHONPATH=$PWD/pingdsp_driver:$PWD/gui
python3 gui/sidescan_export_viewer.py
```

The file dialog opens in `data/` next to the script if it exists (the share-zip
layout), else in `$PINGDSP_SIDESCAN_DIR` if set, else beside the script. First
open of an `.npz` writes a one-time mmap cache (`*.log.npy` + `*.meta.npz`)
beside it for faster reloads; the cache is rebuilt when the `.npz` is newer.

## Knobs (match live GUI)

| Control | Default | Meaning |
| --- | --- | --- |
| Speed compensate | off | Resample along-track so 1 px ≈ one across-track bin (isotropic, optional). |
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

An exporter that reads `sonar/ping` (`pingdsp_msg/Ping3DSS`) from a bag should
write:

| Array | Notes |
| --- | --- |
| `log` | `float16` `(n_pings, n_bins)` = `log1p(|amp|)` combined port\|starboard. |
| `along_m` | Along-track metres (vehicle speed integrated over ping time). |
| `range_res_m` | Across-track metres per bin = `sound_velocity_m_s / (2 · sample_rate_hz)` (≈0.0132 at 1485 m/s, 56.25 kHz). |
| `sample_rate_hz`, `sound_velocity_m_s` | Inputs to `range_res_m`, from `Ping3DSS` (`sample_rate`, `sound_velocity_bulk`). |
| `pulse_range_res_m` | The head's `*_sidescan_range_resolution` (≈0.0165). Pulse range resolution, **not** bin spacing. |

The 3DSS-DX sends one `SidescanPoint` per receive sample, so bin spacing is
`c/(2·fs)`. Do not use `*_sidescan_range_resolution` for it (that is the pulse
resolution and stretches across-track distances by 25 %). The viewer recomputes
spacing from `sample_rate_hz`/`sound_velocity_m_s` when present and falls back to
`range_res_m`, then to 0.0132 m.

Typical width is ~7680 bins (3840/side) or ~5696. Row count = ping count
(up to ~30k). With **Speed compensate** off, the view keeps every ping (1:1).

See also `docs/SIDESCAN_VIEWER.md` for the live ROS GUI / node path.
