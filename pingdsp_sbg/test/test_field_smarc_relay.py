"""
Replay a slice of the Örebro 2026-07-08 capture into the real SBG driver.

The in-process relay test only checks copying. This one checks that a field
pcap's SBG UDP (port 24333) becomes a geographic fix and a compass heading
on ``smarc/*``, which is what the WARA-PS agent publishes.

Skipped when the capture, tshark, or ``sbg_device`` is absent. Override the
capture with ``PINGDSP_FIELD_PCAP``.
"""

import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import time

from geographic_msgs.msg import GeoPoint
import pytest
from sbg_harness import exe, next_domain, RosProbe, wait_until
from std_msgs.msg import Float32

# Opening of a survey capture: enough packets to include the EKF stream on
# UDP 24333 without reading the rest of the file. Point PINGDSP_FIELD_PCAP
# at one (the Jul 8 Örebro slice is 0807_bag5.pcap).
_HEAD_PACKETS = 20000
_FIELD_PCAP = os.environ.get('PINGDSP_FIELD_PCAP', '')


def _field_pcap():
    return _FIELD_PCAP


pytestmark = pytest.mark.skipif(
    not (_FIELD_PCAP and os.path.isfile(_FIELD_PCAP)
         and exe('sbg_to_odom') and exe('sbg_device', 'sbg_driver')
         and shutil.which('tshark') and shutil.which('editcap')),
    reason='set PINGDSP_FIELD_PCAP to a capture with SBG UDP/24333')


def _sbg_payloads(pcap_file):
    """Return UDP/24333 payloads from the opening of ``pcap_file``."""
    with tempfile.TemporaryDirectory(prefix='pingdsp_sbg_slice_') as tmp:
        sliced = os.path.join(tmp, 'head.pcap')
        cut = subprocess.run(
            ['editcap', '-r', pcap_file, sliced, '1-%d' % _HEAD_PACKETS],
            capture_output=True, text=True, check=False)
        if cut.returncode != 0 or not os.path.isfile(sliced):
            raise RuntimeError(
                cut.stderr or cut.stdout or 'editcap produced no slice')
        proc = subprocess.run(
            ['tshark', '-r', sliced, '-Y', 'udp.port == 24333',
             '-T', 'fields', '-e', 'udp.payload'],
            capture_output=True, text=True, check=False)
        if proc.returncode != 0:
            raise RuntimeError(proc.stderr or 'tshark failed')
    payloads = []
    for line in proc.stdout.splitlines():
        line = line.strip()
        if line:
            payloads.append(bytes.fromhex(line.replace(':', '')))
    return payloads


def _free_udp_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def _driver_yaml(port):
    source = Path(__file__).resolve().parents[1] / 'config' / 'sbg_device_udp.yaml'
    text = source.read_text().replace(
        'out_port: 24333', 'out_port: %d' % port)
    handle = tempfile.NamedTemporaryFile(
        mode='w', prefix='sbg_udp_', suffix='.yaml', delete=False)
    handle.write(text)
    handle.close()
    return handle.name


def _popen(args, env):
    return subprocess.Popen(
        args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, env=env)


def _stop(procs):
    for proc in procs:
        if proc.poll() is None:
            proc.terminate()
    for proc in procs:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def _send(payloads, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for payload in payloads:
            sock.sendto(payload, ('127.0.0.1', port))
    finally:
        sock.close()


def test_orebro_slice_publishes_smarc_fix_and_heading():
    """Jul 8 capture yields a Swedish fix and a degree heading on smarc/*."""
    payloads = _sbg_payloads(_field_pcap())
    assert len(payloads) > 20, 'opening of the capture had almost no SBG UDP'

    domain = next_domain()
    saved_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    env = dict(os.environ)
    port = _free_udp_port()
    yaml_path = _driver_yaml(port)
    ns = '/field_smarc'
    procs = []
    probe = None
    try:
        procs.append(_popen([
            exe('sbg_device', 'sbg_driver'), '--ros-args',
            '-r', '__ns:=%s' % ns, '--params-file', yaml_path,
        ], env))
        for name, params in (
                ('sbg_to_odom_initializer', ['require_nav_solution:=true']),
                ('sbg_to_odom', [
                    'use_message_time:=false', 'publish_rate:=20.0']),
                ('smarc_relay', [])):
            args = [exe(name), '--ros-args', '-r', '__ns:=%s' % ns]
            for item in params:
                args += ['-p', item]
            procs.append(_popen(args, env))

        time.sleep(1.5)
        probe = RosProbe('field_smarc_probe')
        latlon = probe.collect(GeoPoint, ns + '/smarc/latlon')
        altitude = probe.collect(Float32, ns + '/smarc/altitude')
        heading = probe.collect(Float32, ns + '/smarc/heading')
        raw_heading = probe.collect(Float32, ns + '/heading')

        last_send = [0.0]

        def _pump():
            now = time.time()
            if now - last_send[0] > 0.4:
                _send(payloads, port)
                last_send[0] = now
            return (len(latlon) > 0 and len(heading) > 0
                    and len(altitude) > 0 and len(raw_heading) > 0)

        assert wait_until(_pump, timeout=25), (
            'no smarc fix/heading from the Örebro slice '
            '(latlon=%d heading=%d)' % (len(latlon), len(heading)))

        fix = latlon[-1]
        assert 55.0 < fix.latitude < 65.0
        assert 10.0 < fix.longitude < 20.0
        assert altitude[-1].data == pytest.approx(fix.altitude)
        assert 0.0 <= heading[-1].data < 360.0
        assert heading[-1].data == pytest.approx(raw_heading[-1].data)
    finally:
        if probe is not None:
            probe.shutdown()
        _stop(procs)
        os.unlink(yaml_path)
        if saved_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = saved_domain
