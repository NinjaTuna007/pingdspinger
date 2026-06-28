#!/usr/bin/env python3
"""Replay a PingDSP network capture (pcap) into the live ROS 2 drivers.

A field capture of the PingDSP rig carries *two* streams on one wire:

  * the 3DSS-DX sonar data as a TCP stream (the sonar is the TCP **server**
    on port 23848; UM002 §12), and
  * the SBG Ellipse INS as sbgECom binary **UDP** broadcasts (port 24333).

To run the full stack offline we have to feed both back to the real drivers
*on a single clock* so the sonar pings and the INS solution stay aligned.
This script does exactly that:

  * It opens a local TCP server. ``tdss_driver`` connects to it as if it were
    the sonar, and we stream the captured sonar bytes at their original timing.
  * It re-emits the captured SBG datagrams as UDP to ``sbg_device``.

Both are dispatched from one monotonic clock keyed to the capture timestamps,
so the relative timing between sonar and INS is preserved.

Only ``live_sensor.pcap`` in ``network_dump/`` contains *both* streams; the
``asko_survey*`` / ``pingDSP_traffic`` captures have sonar only (use
``--no-sbg`` for those).

Example::

    # tdss_driver connects to 127.0.0.1:23848, sbg_device listens on 0.0.0.0:24333
    python3 replay_pingdsp_pcap.py network_dump/live_sensor.pcap

    # sonar-only capture, no INS
    python3 replay_pingdsp_pcap.py network_dump/asko_survey.pcap --no-sbg
"""

import argparse
import collections
import socket
import subprocess
import sys
import threading
import time

# First 8 bytes of the 20-byte 3DSS-DX preamble ("PING" + magic); enough to
# fingerprint the sonar TCP stream inside a mixed capture.
DX_PREAMBLE_HEX = '50:49:4e:47:27:2b:3a:d8'


def _run_tshark(args):
    """Start a streaming tshark process, exiting cleanly if it is missing."""
    try:
        return subprocess.Popen(
            ['tshark', *args],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    except FileNotFoundError:
        print('Error: tshark not found. Install with: sudo apt install tshark',
              file=sys.stderr)
        sys.exit(1)


def detect_sonar_stream(pcap_file, scan_limit=400):
    """Return (tcp_stream, src_ip) of the dominant DX sonar feed in the capture.

    Tallies the TCP segments whose payload carries the DX preamble and picks the
    (stream, source) pair that appears most often. ``src_ip`` is the sonar (the
    TCP server / data source), so we can later select only its direction.

    We scan the file (no read cap -- the sonar may not start until well into the
    capture) but stop after collecting ``scan_limit`` matching segments, which is
    plenty to identify the dominant stream without reading gigabytes.
    """
    proc = _run_tshark([
        '-r', pcap_file,
        '-Y', f'tcp contains {DX_PREAMBLE_HEX}',
        '-T', 'fields', '-e', 'tcp.stream', '-e', 'ip.src',
    ])
    counts = collections.Counter()
    for line in proc.stdout:
        parts = line.rstrip('\n').split('\t')
        if len(parts) < 2 or not parts[0] or not parts[1]:
            continue
        counts[(parts[0], parts[1])] += 1
        if sum(counts.values()) >= scan_limit:
            break
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
    if not counts:
        return None, None
    (stream, src), _ = counts.most_common(1)[0]
    return int(stream), src


def iter_events(pcap_file, want_sonar, want_sbg, sonar_stream, sonar_src,
                sbg_port, start_offset):
    """Yield (t_rel, kind, payload) in capture order via a single tshark pass.

    ``kind`` is ``'tcp'`` (sonar bytes) or ``'udp'`` (SBG datagram). TCP
    desegmentation is disabled so each segment is emitted at its own capture
    time, which is what we want for paced streaming.
    """
    filters = []
    if want_sbg:
        filters.append(f'(udp.dstport == {sbg_port} && udp.length > 8)')
    if want_sonar:
        filters.append(f'(tcp.stream == {sonar_stream} && ip.src == {sonar_src})')
    display_filter = ' || '.join(filters)

    proc = _run_tshark([
        '-r', pcap_file,
        '-o', 'tcp.desegment_tcp_streams:FALSE',
        '-Y', display_filter,
        '-T', 'fields',
        '-e', 'frame.time_relative',
        '-e', 'udp.payload',
        '-e', 'tcp.payload',
    ])
    for line in proc.stdout:
        parts = line.rstrip('\n').split('\t')
        if len(parts) < 3 or not parts[0]:
            continue
        try:
            t_rel = float(parts[0])
        except ValueError:
            continue
        if t_rel < start_offset:
            continue
        udp_hex, tcp_hex = parts[1], parts[2]
        if udp_hex:
            payload, kind = udp_hex, 'udp'
        elif tcp_hex:
            payload, kind = tcp_hex, 'tcp'
        else:
            continue
        try:
            data = bytes.fromhex(payload.replace(':', ''))
        except ValueError:
            continue
        if data:
            yield (t_rel, kind, data)
    proc.wait()
    err = proc.stderr.read()
    if proc.returncode not in (0, None) and err:
        print(f'tshark warning: {err.strip()}', file=sys.stderr)


class SonarTcpServer:
    """Local TCP server that hands captured sonar bytes to a single client.

    ``tdss_driver`` is the client; it reconnects if dropped, so an acceptor
    thread always keeps the most recent connection available to the sender.
    """

    def __init__(self, host, port):
        """Bind and start listening immediately, with a background acceptor."""
        self._lock = threading.Lock()
        self._client = None
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.listen(1)
        self._running = True
        self._connected = threading.Event()
        self._thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._thread.start()
        print(f'Sonar TCP server listening on {host}:{port} '
              '(tdss_driver should connect here)')

    def _accept_loop(self):
        while self._running:
            try:
                conn, addr = self._sock.accept()
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                old = self._client
                self._client = conn
            if old is not None:
                try:
                    old.close()
                except OSError:
                    pass
            print(f'Sonar client connected from {addr[0]}:{addr[1]}')
            self._connected.set()

    def wait_for_client(self, timeout):
        """Block until a client connects (or timeout). Returns True if connected."""
        return self._connected.wait(timeout)

    def send(self, data):
        """Send bytes to the current client; drop it on error so it can retry."""
        with self._lock:
            client = self._client
        if client is None:
            return False
        try:
            client.sendall(data)
            return True
        except OSError:
            with self._lock:
                if self._client is client:
                    self._client = None
                    self._connected.clear()
            try:
                client.close()
            except OSError:
                pass
            return False

    def close(self):
        """Stop accepting and close the listening socket plus any client."""
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass
        with self._lock:
            if self._client is not None:
                try:
                    self._client.close()
                except OSError:
                    pass


def replay(args):
    """Drive the sonar TCP + SBG UDP replay from one capture-relative clock."""
    udp_sock = None
    server = None
    udp_target = (args.sbg_target, args.sbg_udp_port)
    sonar_stream, sonar_src = args.sonar_stream, args.sonar_src

    # Open the TCP server FIRST so tdss_driver (which only retries for ~2s before
    # giving up) can connect immediately; detection / waiting happens after.
    if args.sonar:
        server = SonarTcpServer(args.tcp_listen_host, args.tcp_listen_port)
        if sonar_stream is None or sonar_src is None:
            print('Detecting sonar TCP stream...')
            det_stream, det_src = detect_sonar_stream(args.pcap_file)
            if det_stream is None:
                print('No DX sonar preamble found. Use --sonar-stream/--sonar-src '
                      'to force, or --no-sonar for an SBG-only capture.',
                      file=sys.stderr)
                server.close()
                sys.exit(1)
            sonar_stream = sonar_stream if sonar_stream is not None else det_stream
            sonar_src = sonar_src if sonar_src is not None else det_src
            print(f'Sonar stream = tcp.stream {sonar_stream}, source {sonar_src}')

    if args.sbg:
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    if args.sonar:
        print(f'Waiting up to {args.connect_timeout:.0f}s for the sonar driver '
              'to connect before starting the clock...')
        if not server.wait_for_client(args.connect_timeout):
            print('No sonar client yet; starting anyway (driver may connect late).')

    loop = 0
    try:
        while True:
            loop += 1
            if args.loop:
                print(f'--- replay pass {loop} ---')
            events = iter_events(
                args.pcap_file, args.sonar, args.sbg,
                sonar_stream, sonar_src, args.sbg_udp_port, args.start_offset)

            wall0 = None
            cap0 = None
            n_tcp = n_udp = tcp_bytes = udp_bytes = 0
            t_report = time.monotonic()
            for t_rel, kind, data in events:
                if wall0 is None:
                    wall0 = time.monotonic()
                    cap0 = t_rel
                target = wall0 + (t_rel - cap0) / args.speed
                now = time.monotonic()
                if target > now:
                    time.sleep(target - now)
                if kind == 'udp':
                    if udp_sock is not None:
                        udp_sock.sendto(data, udp_target)
                        n_udp += 1
                        udp_bytes += len(data)
                elif server is not None:
                    if server.send(data):
                        n_tcp += 1
                        tcp_bytes += len(data)
                if time.monotonic() - t_report > 2.0:
                    t_report = time.monotonic()
                    print(f'  sonar {n_tcp} seg/{tcp_bytes / 1e6:.1f} MB | '
                          f'sbg {n_udp} pkt/{udp_bytes / 1024:.0f} KB', end='\r')
            print(f'\nPass complete: sonar {n_tcp} segments / '
                  f'{tcp_bytes / 1e6:.1f} MB, sbg {n_udp} packets / '
                  f'{udp_bytes / 1024:.0f} KB')
            if not args.loop:
                break
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        if udp_sock is not None:
            udp_sock.close()
        if server is not None:
            server.close()


def main():
    """Parse arguments and start the replay."""
    parser = argparse.ArgumentParser(
        description='Replay a PingDSP pcap (sonar TCP + SBG UDP) to the drivers')
    parser.add_argument('pcap_file', help='Path to the .pcap/.pcapng capture')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Playback speed multiplier (1.0 = realtime)')
    parser.add_argument('--loop', action='store_true',
                        help='Replay the capture on repeat')
    parser.add_argument('--start-offset', type=float, default=0.0,
                        help='Skip events before this capture-relative time (s). '
                             'In live_sensor.pcap the sonar starts ~18.4s in.')
    # Sonar (TCP) options.
    parser.add_argument('--no-sonar', dest='sonar', action='store_false',
                        help='Do not replay the sonar TCP stream')
    parser.add_argument('--tcp-listen-host', default='127.0.0.1',
                        help='Address for the local sonar TCP server')
    parser.add_argument('--tcp-listen-port', type=int, default=23848,
                        help='Port for the local sonar TCP server (default 23848)')
    parser.add_argument('--sonar-stream', type=int, default=None,
                        help='Force a tcp.stream index (default: auto-detect)')
    parser.add_argument('--sonar-src', default=None,
                        help='Force the sonar source IP (default: auto-detect)')
    parser.add_argument('--connect-timeout', type=float, default=60.0,
                        help='Seconds to wait for the driver before starting')
    # SBG (UDP) options.
    parser.add_argument('--no-sbg', dest='sbg', action='store_false',
                        help='Do not replay the SBG UDP stream')
    parser.add_argument('--sbg-udp-port', type=int, default=24333,
                        help='SBG UDP port to extract and send to (default 24333)')
    parser.add_argument('--sbg-target', default='127.0.0.1',
                        help='Host to send SBG datagrams to (default 127.0.0.1)')
    args = parser.parse_args()

    if not args.sonar and not args.sbg:
        parser.error('nothing to replay: --no-sonar and --no-sbg are mutually exclusive')

    replay(args)


if __name__ == '__main__':
    main()
