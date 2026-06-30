#!/usr/bin/env python3
"""
Sidescan viewer node for the PingDSP 3DSS-DX.

Historically the ``tdss_driver`` itself rendered a coloured sidescan waterfall
and published it as a ``sensor_msgs/Image`` on ``sonar/intensity``. That image
is large (BGR, up to ~1000x2000x3 bytes per message) and bloats every bag even
though it is purely a visualisation derived from data already present in the
raw ``Ping3DSS`` samples.

This node moves that rendering *out* of the recording path. It subscribes to
``sonar/ping`` (Ping3DSS, which carries the raw port/starboard sidescan
samples), keeps a rolling buffer of the N most recent pings, and renders the
waterfall on demand:

* publishes ``sonar/sidescan_image`` (sensor_msgs/Image, bgr8) on a timer so it
  can be viewed live in Foxglove/rqt (run it only when you want the image),
* saves a PNG of the current buffer via the ``~/save_image`` service, and
* clears the buffer to rebuild from scratch via the ``~/reset`` service
  (both std_srvs/Trigger).

Rendering is deliberately simple and does **no normalisation**: bin the swath
to a fixed width, blank the near-nadir bins, then map ``log1p(amp)`` through a
**fixed** window ``[log_min, log_max]`` to the colormap. The same transfer
function applies to every ping, so brightness is perfectly consistent.

All knobs are live-tunable via the parameter service (no restart), e.g.::

    ros2 param set /sidescan_viewer_node num_pings 500    # rows kept
    ros2 param set /sidescan_viewer_node log_min 12.0     # raise the black point
    ros2 service call /sidescan_viewer_node/reset std_srvs/srv/Trigger

The actual image maths lives in :mod:`pingdsp_driver.sidescan_image` so it is
unit testable without ROS.
"""

from collections import deque
from datetime import datetime
import os

import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from pingdsp_driver import sidescan_image as ssi
from pingdsp_msg.msg import Ping3DSS


class SidescanViewerNode(Node):
    """Renders a sidescan waterfall image from the N most recent pings."""

    def __init__(self):
        super().__init__('sidescan_viewer_node')

        # How many pings (rows) to keep / render. Kept deliberately small here
        # (vs the GUI's 2048) because this image goes into bags: a 512x1024x3
        # frame is ~1.5 MB, so even at a low rate a long survey bag stays sane.
        self.declare_parameter('num_pings', 512)
        # Rendered width in pixels (raw swath is downsampled to this). Smaller
        # than the GUI both to shrink bags and because more samples per bin
        # average out speckle (a steadier image).
        self.declare_parameter('target_width', 1024)
        # Image publish rate (Hz). <=0 disables live publishing. 2 Hz is plenty
        # for a scrolling waterfall and keeps the bag light.
        self.declare_parameter('publish_rate', 2.0)
        # Fixed log window: log1p(amp) in [log_min, log_max] maps to 0..255.
        # No normalisation - this transfer function is constant across pings.
        # Amplitudes are raw float32 envelope magnitudes (can exceed 1e6), so
        # log1p spans ~7-16; this window sits on the seabed band. It scales with
        # the sonar gain/range settings, so re-tune per dataset if needed.
        # These match the control GUI's default preset so the recorded image
        # looks the same as the live GUI view.
        self.declare_parameter('log_min', 11.5)
        self.declare_parameter('log_max', 15.0)
        # Gamma (<1 brightens mid-tones, >1 darkens).
        self.declare_parameter('gamma', 1.0)
        # Colormap: 'bronze', 'copper' or 'gray'. Bronze reads a touch warmer
        # and gives more tonal separation on the seabed than copper.
        self.declare_parameter('colormap', 'bronze')
        # Blank this many pixels each side of nadir (centre = near-transducer
        # closest-range returns). 0 = keep the full swath including nadir.
        self.declare_parameter('nadir_bins', 0)
        # --- Optional feature-enhancement steps (all off by default; with them
        # off the image is the plain fixed-log render, no normalisation) ---
        # Across-track gain flattening (0=off..1=full): removes the range
        # brightness gradient so targets/shadows pop instead of the nadir glow.
        # Default matches the GUI preset (0.70) - this is the single biggest
        # reason the GUI view looks steadier than a raw fixed-log render.
        self.declare_parameter('flatten_strength', 0.70)
        # CLAHE local-contrast clip limit (0=off; ~2-4 typical): makes local
        # texture stand out without globally blowing out bright areas. Mild by
        # default (matches the GUI preset).
        self.declare_parameter('clahe_clip', 0.5)
        self.declare_parameter('clahe_grid', 8)
        # Median despeckle kernel (0/1=off, odd >=3): kills speckle + streaks.
        self.declare_parameter('despeckle', 0)
        # Topic carrying raw Ping3DSS messages.
        self.declare_parameter('input_topic', 'sonar/ping')
        # Topic for the rendered image.
        self.declare_parameter('output_topic', 'sonar/sidescan_image')
        self.declare_parameter('frame_id', 'sonar')
        # Where to drop saved PNGs.
        self.declare_parameter('save_dir', os.path.expanduser('~/sonar_data'))
        # Save a final PNG automatically when the node shuts down.
        self.declare_parameter('save_on_shutdown', False)

        self.num_pings = int(self.get_parameter('num_pings').value)
        self.target_width = int(self.get_parameter('target_width').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.log_min = float(self.get_parameter('log_min').value)
        self.log_max = float(self.get_parameter('log_max').value)
        self.gamma = float(self.get_parameter('gamma').value)
        self.colormap = self.get_parameter('colormap').value
        self.nadir_bins = int(self.get_parameter('nadir_bins').value)
        self.flatten_strength = float(
            self.get_parameter('flatten_strength').value)
        self.clahe_clip = float(self.get_parameter('clahe_clip').value)
        self.clahe_grid = int(self.get_parameter('clahe_grid').value)
        self.despeckle = int(self.get_parameter('despeckle').value)
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.save_dir = self.get_parameter('save_dir').value
        self.save_on_shutdown = bool(
            self.get_parameter('save_on_shutdown').value)

        # Newest row at index 0 (left of deque). maxlen drops the oldest.
        self.rows = deque(maxlen=self.num_pings)
        self.ping_count = 0

        # Renderer: bin -> log (no normalisation). Nadir blanking is applied at
        # render time on the stacked float image (exactly like the GUI), NOT
        # per-row here, so moving the nadir slider stays consistent across all
        # buffered rows. Hence the processor keeps nadir_bins=0.
        self.proc = ssi.WaterfallProcessor(self.target_width,
                                           log_min=self.log_min,
                                           log_max=self.log_max,
                                           gamma=self.gamma,
                                           colormap=self.colormap,
                                           nadir_bins=0)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            Ping3DSS, self.input_topic, self.ping_callback, 10)
        self.image_pub = self.create_publisher(
            Image, self.output_topic, sensor_qos)
        self.save_srv = self.create_service(
            Trigger, '~/save_image', self.save_image_callback)
        self.reset_srv = self.create_service(
            Trigger, '~/reset', self.reset_callback)

        self.timer = None
        self._start_timer()

        # Let num_pings / target_width / publish_rate be changed live via
        # `ros2 param set` (which is itself the .../set_parameters service).
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info('Sidescan Viewer initialised')
        self.get_logger().info(f'  Input:  {self.input_topic} (Ping3DSS)')
        self.get_logger().info(f'  Output: {self.output_topic} (Image)')
        self.get_logger().info(f'  Window: {self.num_pings} pings, '
                               f'width {self.target_width}px')

    def _start_timer(self):
        """(Re)create the publish timer from ``self.publish_rate``."""
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        if self.publish_rate > 0.0:
            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.publish_image)

    def _resize_buffer(self, new_num_pings):
        """Swap in a new deque with ``new_num_pings`` maxlen, keeping newest."""
        new_rows = deque(self.rows, maxlen=new_num_pings)
        self.rows = new_rows
        self.num_pings = new_num_pings

    def _on_set_parameters(self, params):
        """Validate + apply live parameter changes (the set_parameters svc).

        Rows are cached at a fixed pixel width, so a ``target_width`` change
        clears the buffer (mixed-width rows cannot be stacked); ``num_pings``
        is resized in place keeping the most recent rows.
        """
        for p in params:
            if p.name == 'num_pings':
                if p.value is None or int(p.value) < 1:
                    return SetParametersResult(
                        successful=False,
                        reason='num_pings must be >= 1')
                self._resize_buffer(int(p.value))
                self.get_logger().info(f'num_pings -> {self.num_pings}')
            elif p.name == 'target_width':
                if p.value is None or int(p.value) < 1:
                    return SetParametersResult(
                        successful=False,
                        reason='target_width must be >= 1')
                self.target_width = int(p.value)
                self.proc.set_width(self.target_width)
                self.rows.clear()
                self.get_logger().info(
                    f'target_width -> {self.target_width} (buffer cleared)')
            elif p.name == 'publish_rate':
                if p.value is None or float(p.value) < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='publish_rate must be >= 0')
                self.publish_rate = float(p.value)
                self._start_timer()
                self.get_logger().info(
                    f'publish_rate -> {self.publish_rate} Hz')
            elif p.name == 'log_min':
                self.log_min = float(p.value)
                self.proc.log_min = self.log_min
                self.get_logger().info(f'log_min -> {self.log_min}')
            elif p.name == 'log_max':
                self.log_max = float(p.value)
                self.proc.log_max = self.log_max
                self.get_logger().info(f'log_max -> {self.log_max}')
            elif p.name == 'gamma':
                if p.value is None or float(p.value) <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='gamma must be > 0')
                self.gamma = float(p.value)
                self.proc.gamma = self.gamma
                self.get_logger().info(f'gamma -> {self.gamma}')
            elif p.name == 'colormap':
                name = str(p.value).lower()
                if name not in ('copper', 'bronze', 'gray'):
                    return SetParametersResult(
                        successful=False,
                        reason="colormap must be copper/bronze/gray")
                self.colormap = name
                self.proc.colormap = name
                self.get_logger().info(f'colormap -> {name}')
            elif p.name == 'nadir_bins':
                if p.value is None or int(p.value) < 0:
                    return SetParametersResult(
                        successful=False, reason='nadir_bins must be >= 0')
                self.nadir_bins = int(p.value)
                self.get_logger().info(f'nadir_bins -> {self.nadir_bins}')
            elif p.name == 'flatten_strength':
                if p.value is None or not (0.0 <= float(p.value) <= 1.0):
                    return SetParametersResult(
                        successful=False,
                        reason='flatten_strength must be in [0, 1]')
                self.flatten_strength = float(p.value)
                self.get_logger().info(
                    f'flatten_strength -> {self.flatten_strength}')
            elif p.name == 'clahe_clip':
                if p.value is None or float(p.value) < 0.0:
                    return SetParametersResult(
                        successful=False, reason='clahe_clip must be >= 0')
                self.clahe_clip = float(p.value)
                self.get_logger().info(f'clahe_clip -> {self.clahe_clip}')
            elif p.name == 'clahe_grid':
                if p.value is None or int(p.value) < 1:
                    return SetParametersResult(
                        successful=False, reason='clahe_grid must be >= 1')
                self.clahe_grid = int(p.value)
                self.get_logger().info(f'clahe_grid -> {self.clahe_grid}')
            elif p.name == 'despeckle':
                if p.value is None or int(p.value) < 0:
                    return SetParametersResult(
                        successful=False, reason='despeckle must be >= 0')
                self.despeckle = int(p.value)
                self.get_logger().info(f'despeckle -> {self.despeckle}')
        return SetParametersResult(successful=True)

    def ping_callback(self, msg: Ping3DSS):
        """Convert a ping's raw samples to a binned log row and buffer it."""
        try:
            port = np.asarray(msg.port_sidescan_samples, dtype=np.float32)
            stbd = np.asarray(msg.starboard_sidescan_samples, dtype=np.float32)
            row = self.proc.process_row(port, stbd)
            if row is None:
                return
            self.rows.appendleft(row)
            self.ping_count += 1
        except Exception as e:  # noqa: BLE001 - never let a bad ping kill us
            self.get_logger().error(f'Error buffering ping: {e}')

    def render(self):
        """Build the current waterfall image (newest on top) or None.

        Pipeline (all enhancement steps optional / off by default): stack the
        buffered log rows -> [across-track flatten] -> fixed log window + gamma
        -> blank nadir -> [CLAHE] -> [despeckle] -> colormap. The image is
        always ``num_pings`` rows tall (bottom NaN/black until the buffer is
        full) so its dimensions stay constant and viewers do not re-lay-out.
        """
        if not self.rows:
            return None
        log_img = ssi.stack_log_rows(self.rows, self.num_pings)
        if log_img is None:
            return None
        # Exclude the near-nadir band from across-track stats and black it out,
        # on the float image before flattening (matches the GUI render order).
        nb = self.nadir_bins
        if nb > 0 and 2 * nb < log_img.shape[1]:
            c = log_img.shape[1] // 2
            log_img[:, c - nb:c + nb] = np.nan
        if self.flatten_strength > 0.0:
            log_img = ssi.flatten_across_track(log_img, self.flatten_strength)
        gray = ssi.log_to_gray(log_img, self.proc.log_min, self.proc.log_max,
                               self.proc.gamma)
        if self.clahe_clip > 0.0:
            gray = ssi.apply_clahe(gray, self.clahe_clip, self.clahe_grid)
            # CLAHE re-fills the black nadir stripe; blank it again.
            gray = ssi.blank_nadir(gray, nb)
        ds = self.despeckle
        if ds >= 3:
            if ds % 2 == 0:
                ds += 1
            gray = ssi.despeckle_gray(gray, ds)
        return ssi.apply_colormap(gray, self.colormap)

    def publish_image(self):
        """Render and publish the current waterfall as a bgr8 Image."""
        if self.image_pub.get_subscription_count() == 0:
            return
        waterfall = self.render()
        if waterfall is None:
            return

        height, width, _ = waterfall.shape
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = height
        msg.width = width
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = waterfall.tobytes()
        self.image_pub.publish(msg)

    def reset_callback(self, request, response):
        """Trigger service: clear the buffer and rebuild the waterfall fresh."""
        self.rows.clear()
        self.ping_count = 0
        response.success = True
        response.message = 'Sidescan buffer cleared; rebuilding from scratch.'
        self.get_logger().info(response.message)
        return response

    def save_image_callback(self, request, response):
        """Trigger service: write the current waterfall to a PNG."""
        path = self.save_png()
        if path is None:
            response.success = False
            response.message = 'No pings buffered yet; nothing to save.'
        else:
            response.success = True
            response.message = f'Saved sidescan image to {path}'
            self.get_logger().info(response.message)
        return response

    def save_png(self):
        """Write the current waterfall to disk; return path or None."""
        waterfall = self.render()
        if waterfall is None:
            return None
        os.makedirs(self.save_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.save_dir, f'sidescan_{stamp}.png')
        # cv2 is the lightest dependency that is already available; the
        # waterfall is already BGR which is exactly what imwrite expects.
        import cv2
        cv2.imwrite(path, waterfall)
        return path

    def destroy_node(self):
        if self.save_on_shutdown:
            try:
                path = self.save_png()
                if path:
                    self.get_logger().info(f'Saved sidescan image to {path}')
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'Failed to save on shutdown: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SidescanViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
