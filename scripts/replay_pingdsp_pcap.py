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
import os
import select
import socket
import subprocess
import sys
import termios
import threading
import time
import tty

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


class TcpStreamReassembler:
    """Reconstruct a faithful TCP byte stream from captured segments.

    A pcap is a recording of *packets*, not the reassembled stream that a real
    TCP receiver would have seen. Replaying raw ``tcp.payload`` in capture order
    therefore reintroduces artefacts the live sonar link never had:

      * **retransmissions** appear twice (the real receiver discards the
        duplicate by sequence number), and
      * **out-of-order** segments arrive in the wrong order.

    Both corrupt the sonar frame boundaries and force the driver to resync,
    dropping pings that were never actually lost. This class orders segments by
    TCP sequence number, drops duplicate/overlapping bytes, and emits the exact
    in-order byte stream - so the only remaining desyncs correspond to genuine
    *capture-time* drops (bytes tcpdump never recorded), which are physically
    unrecoverable.

    It streams: only out-of-order/gap segments are buffered (bounded by
    ``max_reorder_bytes`` *and* ``max_reorder_secs`` so a hole never stalls the
    replay by volume or by time), and each emitted chunk keeps the capture
    timestamp of the segment that first delivered it, so pacing is preserved.
    """

    _WRAP = 1 << 32
    _HALF = 1 << 31

    def __init__(self, max_reorder_bytes=1024 * 1024, max_reorder_secs=2.0):
        """Set up empty reassembly state and statistics counters.

        A missing segment is declared a genuine (unrecoverable) capture hole and
        skipped once EITHER bound is exceeded while waiting for it:

        ``max_reorder_bytes`` - how many bytes we buffer past the hole. ~1 MB
        (< 1 s of sonar) comfortably absorbs LAN retransmit/reorder windows.

        ``max_reorder_secs`` - how far (in capture time) the newest buffered
        segment may sit past the hole. This bounds the stall in *time* as well as
        volume, so if the stream slows or nearly ends right after a hole (few
        bytes follow, so the byte bound never trips) the replay still moves on
        within a couple seconds instead of stalling until end-of-capture.
        """
        self.next_seq = None
        self.pending = {}          # unwrapped_seq -> (payload, t_rel)
        self.pending_bytes = 0
        self.max_reorder_bytes = max_reorder_bytes
        self.max_reorder_secs = max_reorder_secs
        # Capture time of the segment that first exposed the current hole, and
        # the newest capture time seen so far (monotonic-ish across the stream).
        self._hole_t = None
        self._latest_t = None
        # Diagnostics.
        self.dup_segments = 0
        self.dup_bytes = 0
        self.reordered = 0
        self.holes = 0
        self.hole_bytes = 0
        self.emitted_bytes = 0

    def _unwrap(self, raw):
        """Map a 32-bit raw seq into the monotonic window around next_seq.

        Survives 32-bit sequence wraparound on long streams and lets
        retransmissions (which land just below ``next_seq``) be recognised.
        """
        if self.next_seq is None:
            return raw
        s = raw
        ref = self.next_seq
        while s < ref - self._HALF:
            s += self._WRAP
        while s >= ref + self._HALF:
            s -= self._WRAP
        return s

    def push(self, raw_seq, payload, t_rel):
        """Feed one captured segment; return a list of (t_rel, chunk) to emit."""
        out = []
        if not payload:
            return out
        if self._latest_t is None or t_rel > self._latest_t:
            self._latest_t = t_rel
        seq = self._unwrap(raw_seq)
        if self.next_seq is None:
            self.next_seq = seq
        self._accept(seq, payload, t_rel, out)
        self._drain(out)
        # While a segment is still missing we buffer everything past it. Declare
        # it a genuine (unrecoverable) capture hole and move on once EITHER we
        # have buffered too many bytes OR too much capture time has elapsed since
        # the hole appeared. Each skip restarts the time budget so a run of holes
        # is cleared segment-by-segment without stalling.
        while self.pending:
            if self._hole_t is None:
                self._hole_t = self._latest_t
            overdue = (self._latest_t - self._hole_t) > self.max_reorder_secs
            if self.pending_bytes > self.max_reorder_bytes or overdue:
                self._skip_gap(out)
                self._drain(out)
                self._hole_t = self._latest_t
            else:
                break
        if not self.pending:
            self._hole_t = None
        return out

    def _accept(self, seq, payload, t_rel, out):
        end = seq + len(payload)
        if end <= self.next_seq:
            self.dup_segments += 1
            self.dup_bytes += len(payload)
            return
        if seq < self.next_seq:
            trim = self.next_seq - seq
            self.dup_bytes += trim
            payload = payload[trim:]
            seq = self.next_seq
        if seq == self.next_seq:
            out.append((t_rel, payload))
            self.emitted_bytes += len(payload)
            self.next_seq = seq + len(payload)
        else:
            prev = self.pending.get(seq)
            if prev is not None:
                if len(payload) <= len(prev[0]):
                    self.dup_segments += 1
                    self.dup_bytes += len(payload)
                    return
                self.pending_bytes -= len(prev[0])
            else:
                self.reordered += 1
            self.pending[seq] = (payload, t_rel)
            self.pending_bytes += len(payload)

    def _drain(self, out):
        while self.pending:
            seq = min(self.pending)
            if seq > self.next_seq:
                break
            payload, t_rel = self.pending.pop(seq)
            self.pending_bytes -= len(payload)
            end = seq + len(payload)
            if end <= self.next_seq:
                self.dup_bytes += len(payload)
                continue
            if seq < self.next_seq:
                payload = payload[self.next_seq - seq:]
            out.append((t_rel, payload))
            self.emitted_bytes += len(payload)
            self.next_seq += len(payload)

    def _skip_gap(self, out):
        if not self.pending:
            return
        seq = min(self.pending)
        hole = seq - self.next_seq
        if hole > 0:
            self.holes += 1
            self.hole_bytes += hole
            self.next_seq = seq

    def flush(self):
        """Emit any buffered segments at end-of-capture, accepting real holes."""
        out = []
        while self.pending:
            seq = min(self.pending)
            if seq > self.next_seq:
                self.holes += 1
                self.hole_bytes += seq - self.next_seq
                self.next_seq = seq
            payload, t_rel = self.pending.pop(seq)
            self.pending_bytes -= len(payload)
            end = seq + len(payload)
            if end <= self.next_seq:
                continue
            if seq < self.next_seq:
                payload = payload[self.next_seq - seq:]
            out.append((t_rel, payload))
            self.emitted_bytes += len(payload)
            self.next_seq += len(payload)
        return out

    def stats_line(self):
        """One-line human summary of what reassembly changed vs. raw replay."""
        return (f'reassembly: {self.emitted_bytes / 1e6:.1f} MB emitted, '
                f'{self.dup_segments} dup segs ({self.dup_bytes / 1024:.0f} KB '
                f'deduped), {self.reordered} reordered, '
                f'{self.holes} capture holes ({self.hole_bytes / 1024:.0f} KB '
                f'never recorded)')


def iter_events(pcap_file, want_sonar, want_sbg, sonar_stream, sonar_src,
                sbg_port, start_offset):
    """Yield (t_rel, kind, payload) via a single tshark pass.

    ``kind`` is ``'tcp'`` (sonar bytes) or ``'udp'`` (SBG datagram). SBG UDP
    datagrams are emitted as-is at their capture time. Sonar TCP segments are
    run through :class:`TcpStreamReassembler` so retransmissions/reordering in
    the capture do not corrupt the stream the driver sees; each emitted sonar
    chunk keeps the timestamp of the segment that delivered it, preserving
    pacing. TCP desegmentation stays off so we get one row per captured segment
    (with its sequence number) rather than tshark's own reassembly.
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
        '-e', 'tcp.seq_raw',
    ])
    reasm = TcpStreamReassembler()
    try:
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
            udp_hex = parts[1]
            tcp_hex = parts[2]
            seq_raw = parts[3] if len(parts) > 3 else ''
            if udp_hex:
                try:
                    data = bytes.fromhex(udp_hex.replace(':', ''))
                except ValueError:
                    continue
                if data:
                    yield (t_rel, 'udp', data)
            elif tcp_hex and seq_raw:
                try:
                    data = bytes.fromhex(tcp_hex.replace(':', ''))
                    raw_seq = int(seq_raw)
                except ValueError:
                    continue
                for t_emit, chunk in reasm.push(raw_seq, data, t_rel):
                    yield (t_emit, 'tcp', chunk)

        for t_emit, chunk in reasm.flush():
            yield (t_emit, 'tcp', chunk)

        if want_sonar:
            print(f'  {reasm.stats_line()}', file=sys.stderr)

        proc.wait()
        err = proc.stderr.read()
        if proc.returncode not in (0, None) and err:
            print(f'tshark warning: {err.strip()}', file=sys.stderr)
    finally:
        # Ensure tshark is torn down even if the consumer closes us early
        # (e.g. a mid-pass reset/quit), so no orphaned scan lingers.
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
        for stream in (proc.stdout, proc.stderr):
            try:
                stream.close()
            except OSError:
                pass


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


class PlaybackControls:
    """Thread-safe playback clock + keyboard control for the replay.

    Maps capture time to wall (monotonic) time via a single anchor pair so the
    speed can change and playback can pause/reset without desyncing the sonar and
    SBG streams (they share this one clock). A background thread reads single
    keypresses from the controlling TTY:

        up / down : speed up / slow down (x1.25 per press)
        space     : play / pause
        d         : reset to the default (launch) speed
        l         : toggle looping
        r         : restart the current pass from the beginning
        q         : quit the replay

    If stdin is not a TTY (e.g. piped/non-interactive), the key thread is simply
    not started and playback runs with the launch settings.
    """

    _MIN_SPEED = 0.05
    _MAX_SPEED = 64.0
    _STEP = 1.25

    def __init__(self, speed=1.0, loop=False):
        """Initialise the play clock with the launch speed and loop setting."""
        self._lock = threading.Lock()
        self._speed = self._clamp(speed)
        self._default_speed = self._clamp(speed)
        self._paused = False
        self._loop = loop
        self._reset = False
        self._quit = False
        self._cap_anchor = 0.0
        self._wall_anchor = time.monotonic()
        self._key_thread = None
        self._key_stop = threading.Event()

    def _clamp(self, s):
        """Keep a speed multiplier within the sane playback range."""
        return max(self._MIN_SPEED, min(self._MAX_SPEED, s))

    # --- play clock ------------------------------------------------------
    def start(self, cap0):
        """Anchor the play clock to the first event's capture time."""
        with self._lock:
            self._cap_anchor = cap0
            self._wall_anchor = time.monotonic()

    def _cap_now_locked(self):
        """Return the play clock's capture-time position (lock held)."""
        if self._paused:
            return self._cap_anchor
        return (self._cap_anchor
                + (time.monotonic() - self._wall_anchor) * self._speed)

    def _reanchor_locked(self):
        """Freeze the current position into the anchor (before changing rate)."""
        self._cap_anchor = self._cap_now_locked()
        self._wall_anchor = time.monotonic()

    def wait_until(self, t_cap):
        """Block until the play clock reaches capture time ``t_cap``.

        Returns 'go' when reached, or 'reset'/'quit' if a control interrupts.
        """
        while True:
            with self._lock:
                if self._quit:
                    return 'quit'
                if self._reset:
                    self._reset = False
                    return 'reset'
                if self._paused:
                    sleep_t = 0.05
                else:
                    cn = self._cap_now_locked()
                    if cn >= t_cap:
                        return 'go'
                    sleep_t = min(0.02, (t_cap - cn) / self._speed)
            time.sleep(max(0.001, sleep_t))

    @property
    def loop(self):
        """Return True if the replay should restart after a pass completes."""
        with self._lock:
            return self._loop

    # --- control actions (invoked from the key thread) -------------------
    def faster(self):
        """Increase playback speed one step."""
        with self._lock:
            self._reanchor_locked()
            self._speed = self._clamp(self._speed * self._STEP)

    def slower(self):
        """Decrease playback speed one step."""
        with self._lock:
            self._reanchor_locked()
            self._speed = self._clamp(self._speed / self._STEP)

    def reset_speed(self):
        """Return to the launch (default) playback speed."""
        with self._lock:
            self._reanchor_locked()
            self._speed = self._default_speed

    def toggle_pause(self):
        """Pause or resume playback, keeping the clock continuous."""
        with self._lock:
            self._reanchor_locked()
            self._paused = not self._paused

    def toggle_loop(self):
        """Flip looping on/off for when the current pass ends."""
        with self._lock:
            self._loop = not self._loop

    def request_reset(self):
        """Ask the main loop to restart the current pass from the start."""
        with self._lock:
            self._reset = True

    def request_quit(self):
        """Ask the replay to stop."""
        with self._lock:
            self._quit = True
        self._key_stop.set()

    def status_line(self):
        """Human-readable one-liner describing the current control state."""
        with self._lock:
            state = 'PAUSED' if self._paused else 'PLAY'
            return (f'[{state}] speed x{self._speed:.2f} '
                    f'loop {"on" if self._loop else "off"}')

    # --- keyboard listener ----------------------------------------------
    def start_key_listener(self):
        """Start the TTY key reader thread (no-op if stdin is not a TTY)."""
        if not sys.stdin.isatty():
            print('(stdin is not a TTY - keyboard controls disabled)')
            return
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

    def _key_loop(self):
        """Read single keypresses in cbreak mode and dispatch controls."""
        fd = sys.stdin.fileno()
        try:
            old = termios.tcgetattr(fd)
        except termios.error:
            return
        try:
            tty.setcbreak(fd)
            while not self._key_stop.is_set():
                r, _, _ = select.select([fd], [], [], 0.2)
                if not r:
                    continue
                ch = os.read(fd, 1)
                if not ch:
                    continue
                if ch == b'\x1b':  # escape - maybe an arrow-key sequence
                    r2, _, _ = select.select([fd], [], [], 0.02)
                    seq = os.read(fd, 2) if r2 else b''
                    if seq == b'[A':
                        self.faster()
                    elif seq == b'[B':
                        self.slower()
                    else:
                        continue
                else:
                    c = ch.decode('ascii', 'ignore').lower()
                    if c == ' ':
                        self.toggle_pause()
                    elif c == 'd':
                        self.reset_speed()
                    elif c == 'l':
                        self.toggle_loop()
                    elif c == 'r':
                        self.request_reset()
                    elif c == 'q':
                        self.request_quit()
                    else:
                        continue
                print(f'  {self.status_line()}          ', file=sys.stderr)
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except termios.error:
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

    ctrl = PlaybackControls(speed=args.speed, loop=args.loop)
    print('Controls: [up/down] speed  [space] play/pause  '
          '[d] default speed  [l] toggle loop  [r] restart  [q] quit')
    ctrl.start_key_listener()

    loop = 0
    quit_requested = False
    try:
        while not quit_requested:
            loop += 1
            if ctrl.loop:
                print(f'--- replay pass {loop} ---')
            events = iter_events(
                args.pcap_file, args.sonar, args.sbg,
                sonar_stream, sonar_src, args.sbg_udp_port, args.start_offset)

            cap0 = None
            n_tcp = n_udp = tcp_bytes = udp_bytes = 0
            t_report = time.monotonic()
            do_reset = False
            try:
                for t_rel, kind, data in events:
                    if cap0 is None:
                        cap0 = t_rel
                        ctrl.start(cap0)
                    action = ctrl.wait_until(t_rel)
                    if action == 'quit':
                        quit_requested = True
                        break
                    if action == 'reset':
                        do_reset = True
                        break
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
                        print(f'  {ctrl.status_line()} | '
                              f'sonar {n_tcp} seg/{tcp_bytes / 1e6:.1f} MB | '
                              f'sbg {n_udp} pkt/{udp_bytes / 1024:.0f} KB',
                              end='\r')
            finally:
                events.close()

            if quit_requested:
                print('\nQuit requested')
                break
            if do_reset:
                print('\nRestart -> replaying from the beginning')
                continue
            print(f'\nPass complete: sonar {n_tcp} segments / '
                  f'{tcp_bytes / 1e6:.1f} MB, sbg {n_udp} packets / '
                  f'{udp_bytes / 1024:.0f} KB')
            if not ctrl.loop:
                break
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        ctrl.request_quit()
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
