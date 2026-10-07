"""Full-stack integration tests for pingdsp_driver.

Skipped automatically unless the package is built/sourced (the harness needs the
installed ``tdss_driver`` / ``sidescan_viewer_node`` executables). The sonar is
emulated by an in-process TCP server streaming synthetic DX frames.
"""

import shutil
import signal

from dx_frame_factory import build_dx_frame
from full_stack_harness import DRIVER_AVAILABLE, exe, Stack, wait_until
import pytest

pytestmark = pytest.mark.skipif(
    not DRIVER_AVAILABLE, reason='pingdsp_driver not built/sourced')


HEAD_SENTENCES = [
    '$GPHDT,90.0,T*0B',
    '$PDNDE,-3.98,-0.63,64,271*6A',
    '$PDSVM,1483.230,20.12,AML,222864*2A',
    '$PDHXT,MCU,037*00',
    '$PDHXP,D1V2,1218,1218,0000,1218,1221,0063,01487*00',
]


def _survey_frames():
    """Return frames with nav, sidescan, 3D and bathymetry populated."""
    return [
        build_dx_frame(
            ping_id=i,
            ascii_sentences=HEAD_SENTENCES,
            port_sidescan=[float(i), float(i + 1), float(i + 2), float(i + 3)],
            starboard_sidescan=[1.0, 2.0, 3.0, 4.0],
            port_sidescan3d=[(10.0, -0.2, 100.0, 37.2), (10.0, -0.3, 50.0, 20.0)],
            starboard_sidescan3d=[(12.0, 0.2, 80.0, 30.0)],
            port_bathy=[(10.0, -0.2, 100.0, 7)],
            starboard_bathy=[(12.0, 0.2, 80.0, 1)],
            range_m=75.0, sv_bulk=1485.0, sample_rate=56250.0,
        )
        for i in range(1, 4)
    ]


def _fields(cloud):
    return [f.name for f in cloud.fields]


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
        cloud = clouds[-1]
        assert cloud.width * cloud.height == 2
        assert _fields(cloud) == ['x', 'y', 'z', 'intensity', 'quality']
        assert cloud.point_step == 20
        import numpy as np
        pts = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 5)
        assert pts[:, 4].tolist() == [7.0, 1.0]
    finally:
        st.stop()


@pytest.mark.skipif(exe('pointcloud_filter') is None,
                    reason='pointcloud_filter not built')
def test_pointcloud_filter_keeps_extra_fields():
    """The filter must republish driver fields beyond xyzi (quality)."""
    from sensor_msgs.msg import PointCloud2

    st = Stack(frames=_survey_frames(), interval=0.05).start_sonar()
    probe = st.start_probe()
    filtered = probe.collect(PointCloud2, '/sonar/bathymetry_filtered')
    st.start_node(exe('pointcloud_filter'),
                  ['-p', 'min_range:=0.5', '-p', 'min_intensity:=0.0'])
    st.start_driver()
    try:
        assert wait_until(lambda: len(filtered) > 0, timeout=20), \
            'no filtered PointCloud2 published'
        cloud = filtered[-1]
        assert _fields(cloud) == ['x', 'y', 'z', 'intensity', 'quality']
        assert cloud.width * cloud.height == 2
        import numpy as np
        pts = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 5)
        assert pts[:, 4].tolist() == [7.0, 1.0]
    finally:
        st.stop()


def test_driver_publishes_sidescan3d_pointcloud():
    from sensor_msgs.msg import PointCloud2

    st = Stack(frames=_survey_frames(), interval=0.05).start_sonar()
    probe = st.start_probe()
    clouds = probe.collect(PointCloud2, '/sonar/sidescan3d')
    st.start_driver()
    try:
        assert wait_until(lambda: len(clouds) > 0, timeout=20), \
            'no sidescan3d PointCloud2 published'
        cloud = clouds[-1]
        assert cloud.width * cloud.height == 3
        assert _fields(cloud) == ['x', 'y', 'z', 'intensity', 'snr']
        import numpy as np
        pts = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 5)
        assert pts[:, 4].tolist() == pytest.approx([37.2, 20.0, 30.0], abs=1e-4)
    finally:
        st.stop()


def test_driver_drops_spliced_frame_and_recovers():
    """A frame carrying the next frame's bytes is dropped whole; stream resyncs."""
    from pingdsp_msg.msg import Ping3DSS
    from sensor_msgs.msg import PointCloud2

    frames = _survey_frames()                      # pings 1, 2, 3
    a, b, c = frames
    # Lose 300 bytes from the tail of ping 1: the driver reads ping 1's
    # advertised length and gets ping 2's header + start spliced in; the rest
    # of ping 2 is then garbage until ping 3's preamble.
    spliced = a[:-300] + b
    st = Stack(frames=[spliced, c], interval=0.05).start_sonar()
    probe = st.start_probe()
    pings = probe.collect(Ping3DSS, '/sonar/ping')
    clouds = probe.collect(PointCloud2, '/sonar/bathymetry')
    st.start_driver()
    try:
        assert wait_until(lambda: len(pings) >= 3, timeout=20), 'driver did not recover'
        ids = {int(p.ping_number) for p in pings}
        assert ids == {3}, f'corrupt pings leaked: {ids}'
        import numpy as np
        for cl in clouds:
            pts = np.frombuffer(cl.data, dtype='<f4').reshape(-1, 5)
            assert np.all(np.isfinite(pts)) and np.abs(pts[:, :3]).max() < 100
    finally:
        st.stop()


@pytest.mark.skipif(shutil.which('ros2') is None, reason='ros2 CLI not available')
def test_bag_record_all_captures_every_driver_topic(tmp_path):
    """`ros2 bag record -a` (as the bringup runs it) must get every topic."""
    st = Stack(frames=_survey_frames(), interval=0.05).start_sonar()
    probe = st.start_probe()
    pings = probe.collect(__import__('pingdsp_msg.msg', fromlist=['Ping3DSS']).Ping3DSS,
                          '/sonar/ping')
    rec = st.start_process(['ros2', 'bag', 'record', '-a', '-o', 'bag'], cwd=str(tmp_path))
    st.start_driver()
    try:
        assert wait_until(lambda: len(pings) >= 40, timeout=30), 'driver not streaming'
        rec.send_signal(signal.SIGINT)
        rec.wait(timeout=30)
        import yaml
        meta = yaml.safe_load((tmp_path / 'bag' / 'metadata.yaml').read_text())
        info = meta['rosbag2_bagfile_information']
        counts = {t['topic_metadata']['name']: t['message_count']
                  for t in info['topics_with_message_count']}
        expected = [
            '/sonar/ping', '/sonar/bathymetry', '/sonar/sidescan3d',
            '/sonar/settings', '/sonar/altitude', '/sonar/water_temperature',
            '/sonar/mcu_temperature', '/sonar/nmea', '/sonar/delivery_latency',
            '/sonar/rx_backlog_bytes',
        ]
        missing = [t for t in expected if counts.get(t, 0) == 0]
        assert not missing, f'topics not in bag: {missing}; have {counts}'
        # Latched settings: one message is all there is to record.
        assert counts['/sonar/settings'] >= 1
        # Per-ping topics recorded at the same rate as the ping itself.
        n = counts['/sonar/ping']
        assert n >= 30
        for t in ('/sonar/bathymetry', '/sonar/sidescan3d', '/sonar/altitude'):
            assert abs(counts[t] - n) <= 5, f'{t}: {counts[t]} vs ping {n}'
    finally:
        st.stop()


def test_navsatfix_carries_gst_covariance():
    """$GPGST sigmas become a DIAGONAL_KNOWN ENU covariance on sonar/fix."""
    from sensor_msgs.msg import NavSatFix

    frames = [build_dx_frame(
        ping_id=i,
        ascii_sentences=[
            '$GPGST,084540.10,0.149,0.008,0.004,2.368,0.008,0.004,0.014*5D',
            '$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47',
        ]) for i in range(1, 4)]
    st = Stack(frames=frames, interval=0.05).start_sonar()
    probe = st.start_probe()
    fixes = probe.collect(NavSatFix, '/sonar/fix')
    st.start_driver()
    try:
        assert wait_until(lambda: len(fixes) > 0, timeout=20), 'no NavSatFix'
        fix = fixes[-1]
        assert fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        cov = list(fix.position_covariance)
        assert cov[0] == pytest.approx(0.004 ** 2, rel=1e-4)   # east  = lon sd
        assert cov[4] == pytest.approx(0.008 ** 2, rel=1e-4)   # north = lat sd
        assert cov[8] == pytest.approx(0.014 ** 2, rel=1e-4)   # up    = alt sd
    finally:
        st.stop()


def test_driver_publishes_head_telemetry():
    """$PDNDE altitude, probe/MCU temperatures, and the latched settings."""
    from pingdsp_msg.msg import SonarAltitude, SonarSettings
    from sensor_msgs.msg import Temperature

    st = Stack(frames=_survey_frames(), interval=0.05).start_sonar()
    probe = st.start_probe()
    alts = probe.collect(SonarAltitude, '/sonar/altitude')
    water = probe.collect(Temperature, '/sonar/water_temperature')
    mcu = probe.collect(Temperature, '/sonar/mcu_temperature')
    st.start_driver()
    try:
        assert wait_until(lambda: alts and water and mcu, timeout=20), \
            'head telemetry topics not all published'
        assert alts[-1].nadir_depth == pytest.approx(-3.98, abs=1e-5)
        assert alts[-1].altitude == pytest.approx(3.98, abs=1e-5)
        assert alts[-1].field3 == 64 and alts[-1].field4 == 271
        assert water[-1].temperature == pytest.approx(20.12, abs=1e-5)
        assert mcu[-1].temperature == pytest.approx(37.0)

        # Settings are latched: a subscriber that arrives after the first
        # ping still gets the snapshot, and identical pings do not re-send.
        settings = probe.collect(SonarSettings, '/sonar/settings',
                                 transient_local=True)
        assert wait_until(lambda: len(settings) > 0, timeout=10), \
            'latched SonarSettings not delivered to late subscriber'
        s = settings[-1]
        assert s.range == pytest.approx(75.0)
        assert s.sound_velocity_bulk == pytest.approx(1485.0)
        assert s.sidescan_bin_spacing == pytest.approx(1485.0 / 112500.0)
        assert s.port_transmit_power == 80
        assert s.sonar_id == 'TEST-3DSS'
        import time
        time.sleep(0.5)
        assert len(settings) == 1, 'settings re-published without a change'
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
