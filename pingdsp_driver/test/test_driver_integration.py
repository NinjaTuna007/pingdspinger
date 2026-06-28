"""Full-stack integration tests for pingdsp_driver.

Skipped automatically unless the package is built/sourced (the harness needs the
installed ``tdss_driver`` / ``sidescan_viewer_node`` executables). The sonar is
emulated by an in-process TCP server streaming synthetic DX frames.
"""

from dx_frame_factory import build_dx_frame
from full_stack_harness import DRIVER_AVAILABLE, exe, Stack, wait_until
import pytest

pytestmark = pytest.mark.skipif(
    not DRIVER_AVAILABLE, reason='pingdsp_driver not built/sourced')


def _survey_frames():
    """Return frames with nav, sidescan and bathymetry populated."""
    return [
        build_dx_frame(
            ping_id=i,
            ascii_sentences=['$GPHDT,90.0,T*0B'],
            port_sidescan=[float(i), float(i + 1), float(i + 2), float(i + 3)],
            starboard_sidescan=[1.0, 2.0, 3.0, 4.0],
            port_bathy=[(10.0, -0.2, 100.0)],
            starboard_bathy=[(12.0, 0.2, 80.0)],
        )
        for i in range(1, 4)
    ]


def test_driver_publishes_ping():
    from pingdsp_msg.msg import Ping3DSS

    st = Stack(frames=_survey_frames(), interval=0.05).start_sonar()
    probe = st.start_probe()
    pings = probe.collect(Ping3DSS, '/sonar/ping')
    st.start_driver()
    try:
        ok = wait_until(lambda: len(pings) > 0, timeout=20)
        out = ''
        if not ok and st.driver is not None:
            st.driver.terminate()
            out = st.driver.stdout.read() if st.driver.stdout else ''
        assert ok, f'no Ping3DSS published; driver output:\n{out}'
        msg = pings[-1]
        assert len(msg.port_sidescan_samples) == 4
        assert len(msg.starboard_sidescan_samples) == 4
    finally:
        st.stop()


def test_driver_publishes_bathymetry_pointcloud():
    from sensor_msgs.msg import PointCloud2

    st = Stack(frames=_survey_frames(), interval=0.05).start_sonar()
    probe = st.start_probe()
    clouds = probe.collect(PointCloud2, '/sonar/bathymetry')
    st.start_driver()
    try:
        assert wait_until(lambda: len(clouds) > 0, timeout=20), \
            'no bathymetry PointCloud2 published'
        assert clouds[-1].width * clouds[-1].height > 0
    finally:
        st.stop()


@pytest.mark.skipif(exe('sidescan_viewer_node') is None,
                    reason='sidescan_viewer_node not built')
def test_sidescan_viewer_renders_and_retunes():
    from pingdsp_msg.msg import Ping3DSS
    from sensor_msgs.msg import Image

    st = Stack(frames=[], loop=False).start_sonar()  # no driver needed
    probe = st.start_probe()
    images = probe.collect(Image, '/sonar/sidescan_image')
    pub = probe.node.create_publisher(Ping3DSS, '/sonar/ping', 10)
    st.start_node(
        exe('sidescan_viewer_node'),
        ['-p', 'publish_rate:=10.0', '-p', 'target_width:=8'])
    try:
        # Feed pings; viewer only renders while something subscribes (probe does).
        def _pump():
            msg = Ping3DSS()
            msg.port_sidescan_samples = [10, 20, 30, 40]
            msg.starboard_sidescan_samples = [40, 30, 20, 10]
            pub.publish(msg)
            return len(images) > 0

        assert wait_until(_pump, timeout=20), 'viewer published no image'
        img = images[-1]
        assert img.encoding == 'bgr8'
        assert img.width > 0 and img.height > 0

        # Live retune via the parameter service (no relaunch).
        assert probe.set_param('sidescan_viewer_node', 'num_pings', 50)
        assert probe.set_param('sidescan_viewer_node', 'target_width', 6)
    finally:
        st.stop()
