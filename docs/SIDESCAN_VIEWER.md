# Sidescan viewing

The sidescan waterfall is rendered from the raw `Ping3DSS` samples. It is
intentionally **decoupled from the recording path**: `tdss_driver` used to
publish a rendered image inline, which bloated bags badly. The driver now emits
only the raw samples, and the waterfall is rendered **live for viewing only**.

There are two ways to view it; the GUI tab is the default (and the only one the
bringup starts):

1. **Control GUI "Sidescan" tab (recommended, no topic).** The waterfall is
   drawn directly inside the 3DSS-DX control GUI (`gui/sonar_control_gui.py`)
   and never leaves the process, so **no `sensor_msgs/Image` topic exists** and
   bags stay lean (the raw `Ping3DSS` is already in the bag). Every knob is a
   live slider.
2. **`sidescan_viewer_node` (optional).** A headless node that publishes the
   waterfall as `sensor_msgs/Image` on `sonar/sidescan_image` for Foxglove/rviz.
   Useful for remote viewing, but it **does** create an image topic, so a
   `ros2 bag record -a` will capture it — only run it when you want that. It is
   **not** launched by `pingdsp_bringup.sh`.

Both share the same pure/numpy, unit-tested maths in
`pingdsp_driver/sidescan_image.py`.

## The render pipeline (shared)

* Combine port + starboard into one across-track row and **bin-average** it down
  to the render width (averages out per-sample speckle, fixes the image width).
* Store `log1p(amp)` per ping in a ring buffer of the most recent `num_pings`.
* At display time the buffered log rows are stacked and the transfer is applied
  to the whole waterfall: optional **across-track flatten**, the **fixed**
  `[log_min, log_max]` window + gamma, nadir blanking, optional **CLAHE** +
  **despeckle**, then the colormap. With every enhancement off this is a
  constant per-pixel log map — **no normalisation**, no percentiles, no adaptive
  contrast — so brightness never flickers or "resets". The optional steps only
  add *spatial* corrections.

## 1. Live GUI tab

Started automatically by the bringup (`ENABLE_GUI=true`, the default) in the
`viz` window, or on its own:

```bash
python3 gui/sonar_control_gui.py
```

Open the **Sidescan** tab (it is the first tab). The left side is the live
waterfall (newest ping on top, scaled to fit); the right side has a slider for
every knob plus **Reset**, **Save PNG** and **Pause**:

| Slider | Default | Meaning |
| --- | --- | --- |
| Pings (rows) | 2048 | Rows kept in the waterfall (ring buffer). |
| Width (px) | 2048 | Render width; raw swath is bin-averaged to this (clears buffer). |
| Log min | 11.5 | `log1p(amp)` mapped to black. Raise for more contrast. |
| Log max | 14.5 | `log1p(amp)` mapped to white. Lower for more contrast. |
| Gamma | 1.0 | `<1` brightens mid-tones, `>1` darkens. |
| Nadir bins | 0 | Pixels each side of nadir to blank (0 = keep all). |
| Flatten | 0.0 | Across-track gain flattening, 0=off..1=full (biggest lever; ~0.85 is a good start). |
| CLAHE clip | 0.0 | Local-contrast (CLAHE) clip limit (0 = off, ~2–4 typical). |
| Despeckle | 0 | Median despeckle kernel (0 = off, applied at odd ≥ 3). |
| Refresh (Hz) | 5 | Redraw rate. |
| Colormap | copper | `copper`, `bronze` or `gray`. |

Rendering only runs while the Sidescan tab is visible (and not paused), so the
other control tabs cost nothing. **Save PNG** writes the current full-resolution
waterfall to `~/sonar_data/`.

## 2. Optional headless node

```bash
ros2 launch pingdsp_driver sidescan_viewer.launch
ros2 launch pingdsp_driver sidescan_viewer.launch num_pings:=500 log_min:=12.0
```

Every knob above maps to a node parameter (re-applied live, no relaunch):

```bash
ros2 param set /sidescan_viewer_node flatten_strength 0.85
ros2 param set /sidescan_viewer_node log_min 12.0
ros2 param set /sidescan_viewer_node colormap gray
ros2 service call /sidescan_viewer_node/reset std_srvs/srv/Trigger
ros2 service call /sidescan_viewer_node/save_image std_srvs/srv/Trigger
```

Parameters and defaults live in
`pingdsp_driver/config/sidescan_viewer_params.yaml`; the node publishes
`sonar/sidescan_image` (`bgr8`) at `publish_rate` Hz (`<= 0` disables).

## Tuning notes

> The samples are the raw **float32** `SidescanPoint.amplitude` (envelope
> magnitude after gain/processing) and routinely exceed 1e6, so `log1p` spans
> ~7–16 and the seabed sits ~11.8–14.6. The defaults are tuned onto that band,
> but the absolute amplitude scales with the sonar gain/range settings, so
> **re-tune `log_min`/`log_max` per dataset**. The only smoothing is
> across-track bin-averaging (lower the width to smooth more).
>
> NB: `Ping3DSS.*_sidescan_samples` is `float32[]`. It used to be `uint16[]`,
> which wrapped these >65535 amplitudes mod-65536 into a uniform "wall of
> noise" — if you ever see that again, check for a stale `uint16` cast.
