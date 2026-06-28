# Sidescan viewer

`sidescan_viewer_node` renders a live sidescan waterfall from raw `Ping3DSS`
samples. It is intentionally **decoupled from the driver**: `tdss_driver` used to
publish a rendered image inline, which bloated bags badly. Now the driver only
emits the raw samples, and visualisation is an optional, separately-launched node
that you can run when (and only when) you want to look at the data.

## What it does

* Subscribes to `Ping3DSS` (default `sonar/ping`).
* Buffers the most recent `num_pings` rows in a ring buffer.
* Per ping (`pingdsp_driver/sidescan_image.py`, pure/numpy + unit-tested):
  combines port + starboard into one across-track row, **bin-averages** it down
  to `target_width`, and stores `log1p(amp)` (near-nadir band marked blank).
* At publish time the buffered log rows are stacked and the display transfer is
  applied to the whole waterfall: optional **across-track flatten**, the
  **fixed** `[log_min, log_max]` window + gamma, nadir blanking, optional
  **CLAHE** + **despeckle**, then the colormap. With every enhancement off this
  is a constant per-pixel log map - **no normalisation**, no percentiles, no
  adaptive contrast - so brightness never flickers or "resets". The optional
  steps (see below) only add *spatial* corrections.
* Publishes the stacked waterfall as `sensor_msgs/Image` (`bgr8`) on
  `sonar/sidescan_image` on a timer.
* Optionally saves the final image on shutdown (`save_on_shutdown`, `save_dir`).

## Launch

```bash
ros2 launch pingdsp_driver sidescan_viewer.launch
# overrides:
ros2 launch pingdsp_driver sidescan_viewer.launch num_pings:=500 log_min:=12.0
```

## Services

```bash
# Clear the buffer and rebuild the waterfall from scratch:
ros2 service call /sidescan_viewer_node/reset std_srvs/srv/Trigger
# Save the current waterfall to a PNG (under save_dir):
ros2 service call /sidescan_viewer_node/save_image std_srvs/srv/Trigger
```

## Making features stand out (optional enhancements)

By default the image is the plain fixed-log render (no normalisation). Three
optional, live-toggleable steps make features pop without reintroducing
per-ping flicker - they are spatial corrections on the assembled waterfall, not
temporal scaling:

* **`flatten_strength`** (0=off..1=full) - across-track gain flattening. Removes
  the range-dependent brightness gradient (the nadir glow / bright outer edges)
  by re-levelling every across-track column to the global mean, in the log
  domain *before* the window so nothing is clipped first. This is the single
  biggest lever; ~0.85 is a good start.
* **`clahe_clip`** (0=off, ~2-4 typical) + **`clahe_grid`** - local-contrast
  (CLAHE) equalisation, boosts local texture without globally blowing out the
  bright returns.
* **`despeckle`** (0=off, odd >=3) - median filter that knocks out speckle and
  thin bad-ping streaks.

```bash
ros2 param set /sidescan_viewer_node flatten_strength 0.85   # flatten range gain
ros2 param set /sidescan_viewer_node clahe_clip 2.5          # local contrast
ros2 param set /sidescan_viewer_node despeckle 3             # de-speckle
```

## Runtime tuning (no relaunch)

Every knob is re-applied live through the parameter service:

```bash
ros2 param set /sidescan_viewer_node log_min 12.0     # raise the black point (more contrast)
ros2 param set /sidescan_viewer_node log_max 14.0     # lower the white point (more contrast)
ros2 param set /sidescan_viewer_node nadir_bins 20    # blank more near-nadir pixels
ros2 param set /sidescan_viewer_node colormap gray    # copper | bronze | gray
ros2 param set /sidescan_viewer_node gamma 0.8        # <1 brightens mid-tones
ros2 param set /sidescan_viewer_node target_width 256 # fewer/bigger bins = smoother (clears buffer)
ros2 param set /sidescan_viewer_node num_pings 500    # resizes the ring buffer
```

## Parameters

`pingdsp_driver/config/sidescan_viewer_params.yaml`:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `num_pings` | 2048 | Rows (pings) kept in the waterfall. |
| `target_width` | 2048 | Rendered width in px (raw swath is bin-averaged to this). |
| `publish_rate` | 5.0 | Live image publish rate (Hz); `<= 0` disables. |
| `log_min` | 11.5 | `log1p(amp)` mapped to black. Raise for more contrast. |
| `log_max` | 14.5 | `log1p(amp)` mapped to white. Lower for more contrast. |
| `gamma` | 1.0 | `<1` brightens mid-tones, `>1` darkens. |
| `colormap` | `copper` | `copper`, `bronze` or `gray`. |
| `nadir_bins` | 0 | Pixels each side of nadir (centre) to blank (0=keep all). |
| `flatten_strength` | 0.0 | Across-track gain flattening, 0=off..1=full. |
| `clahe_clip` | 0.0 | CLAHE local-contrast clip limit (0=off). |
| `clahe_grid` | 8 | CLAHE tile grid size per axis. |
| `despeckle` | 0 | Median despeckle kernel (0=off, odd >=3). |
| `input_topic` | `sonar/ping` | `Ping3DSS` input. |
| `output_topic` | `sonar/sidescan_image` | rendered `Image` output. |
| `frame_id` | `sonar` | image header frame. |
| `save_dir` | `~/sonar_data` | where a snapshot is written. |
| `save_on_shutdown` | false | save the final waterfall on exit. |

> The samples are the raw **float32** `SidescanPoint.amplitude` (envelope
> magnitude after gain/processing) and routinely exceed 1e6, so `log1p` spans
> ~7-16 and the seabed sits ~11.8-14.6. The default window is tuned onto that
> band, but the absolute amplitude scales with the sonar gain/range settings,
> so **re-tune `log_min`/`log_max` per dataset**. The only smoothing is
> across-track bin-averaging (lower `target_width` to smooth more).
>
> NB: `Ping3DSS.*_sidescan_samples` is `float32[]`. It used to be `uint16[]`,
> which wrapped these >65535 amplitudes mod-65536 into a uniform "wall of
> noise" - if you ever see that again, check for a stale `uint16` cast.

View it in Foxglove or rviz2 by subscribing to `/sonar/sidescan_image`.
