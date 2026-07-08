#!/usr/bin/env python3
"""TCP client for the 3DSS-DX sonar.

Handles the TCP socket connection to the 3DSS-DX sonar and frames the incoming
byte stream into pings.

Design note - losslessness
--------------------------
TCP already guarantees a reliable, in-order, gap-free byte stream while the
connection is up. The only way this client can *lose* framing is by throwing
away bytes it has already received. Earlier versions discarded a partially
received frame whenever a socket ``recv`` timed out mid-frame, which
desynchronised the stream and forced an expensive byte-by-byte resync (dropping
whole pings -> "black bars" on the sidescan).

This implementation keeps a single persistent receive buffer on the client
instance. Bytes are appended to it as they arrive and removed only when a
*complete* frame is consumed (or, during a resync, when they are confirmed to
be pre-preamble garbage). A timeout never discards anything - it simply returns
``None`` so the reader loop can check for shutdown and call again, resuming
exactly where it left off. On a healthy live link the stream stays perfectly
aligned and ``resync_count`` stays at zero.
"""

import array
import logging
import socket
from typing import Optional, Tuple

# FIONREAD lets us read how many bytes are queued in the kernel socket receive
# buffer (Linux). Used purely for the backlog diagnostic; guarded so the client
# still works if unavailable.
try:
    import fcntl
    import termios
    _FIONREAD = termios.FIONREAD
except Exception:  # pragma: no cover - non-Linux / missing termios
    fcntl = None
    _FIONREAD = None

from .dx_structures import DxData, DxHeader

# 16-byte frame preamble ("PING'+:..." magic) that marks the start of every
# 3DSS-DX message.
PREAMBLE = bytes([0x50, 0x49, 0x4e, 0x47, 0x27, 0x2b, 0x3a, 0xd8,
                  0x74, 0x2a, 0x1c, 0x33, 0xe9, 0xb0, 0x73, 0xb1])

# Hard upper bound on a single frame's payload. Real pings are at most a few
# MB; anything larger means the header was misaligned/corrupt. Reading (and
# parsing) a multi-GB "frame" would stall the reader thread, so we reject it
# and resync instead.
MAX_DATA_COUNT = 32 * 1024 * 1024  # 32 MB

# How many bytes to request per recv() and the desired kernel receive buffer.
_RECV_CHUNK = 262144          # 256 KB per syscall
_SO_RCVBUF = 8 * 1024 * 1024  # ask the kernel for an 8 MB socket buffer


class TcpClient:
    """TCP client for connecting to a 3DSS-DX sonar.

    Maintains a persistent receive buffer so no received byte is ever dropped
    except a fully consumed frame or confirmed pre-preamble garbage.
    """

    def __init__(self, host: str, port: int, timeout: float = 5.0):
        """Initialise the client (does not connect yet).

        Args:
            host: IP address or hostname of the 3DSS-DX sonar.
            port: TCP port (3DSS-DX data stream, 23848).
            timeout: Socket timeout in seconds.
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None
        self.connected = False

        # Persistent receive buffer - the heart of the lossless design.
        self._buf = bytearray()

        # Diagnostics: on a live TCP link these should stay at zero. Any
        # non-zero value means the stream desynchronised (corrupt capture,
        # reconnect mid-frame, or a real link fault).
        self.resync_count = 0
        self.bytes_discarded = 0

        self.logger = logging.getLogger('TcpClient')

    def connect(self) -> bool:
        """Connect to the 3DSS-DX sonar.

        Returns:
            True if connection successful, False otherwise.
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            try:
                self.socket.setsockopt(
                    socket.SOL_SOCKET, socket.SO_RCVBUF, _SO_RCVBUF)
            except OSError:
                pass  # best-effort; kernel may clamp
            self.socket.connect((self.host, self.port))
            try:
                self.socket.setsockopt(
                    socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
            self.connected = True
            # A fresh connection starts a fresh stream; drop any stale bytes.
            self._buf.clear()
            self.logger.info(f"Connected to {self.host}:{self.port}")
            return True

        except socket.timeout:
            self.logger.error(f"Connection timeout to {self.host}:{self.port}")
            self.connected = False
            return False

        except socket.error as e:
            self.logger.error(f"Connection error: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Close the TCP connection."""
        if self.socket:
            try:
                self.socket.close()
            except OSError:
                pass
            self.socket = None
            self.connected = False
            self.logger.info("Disconnected")

    # ------------------------------------------------------------------ #
    # Low-level buffered reads
    # ------------------------------------------------------------------ #
    def _recv_more(self) -> str:
        """Pull one chunk from the socket into the buffer.

        Returns one of:
            'data'    - bytes were appended
            'timeout' - recv timed out; buffer left untouched
            'closed'  - peer closed or socket error (sets connected=False)
        """
        if not self.socket:
            self.connected = False
            return 'closed'
        try:
            chunk = self.socket.recv(_RECV_CHUNK)
        except socket.timeout:
            return 'timeout'
        except socket.error as e:
            self.logger.error(f"Socket error during recv: {e}")
            self.connected = False
            return 'closed'
        if not chunk:
            self.logger.warning("Connection closed by remote")
            self.connected = False
            return 'closed'
        self._buf.extend(chunk)
        return 'data'

    def _ensure(self, n: int) -> Optional[bool]:
        """Ensure at least ``n`` bytes are buffered.

        Returns:
            True  - at least ``n`` bytes are available.
            None  - not yet (recv timed out); buffer is preserved, retry later.
            False - the connection closed/errored.
        """
        while len(self._buf) < n:
            r = self._recv_more()
            if r == 'timeout':
                return None
            if r == 'closed':
                return False
        return True

    def pending_bytes(self) -> int:
        """Approx bytes waiting to be processed: kernel socket queue + buffer.

        A persistently growing value means this client is not draining the
        stream as fast as it arrives - i.e. the driver is falling behind
        locally (CPU-bound processing). Note this is distinct from upstream
        delivery lag (the 3DSS control PC sending late), which instead shows up
        as a large-but-stable delivery latency with this backlog near zero.
        """
        kernel = 0
        sock = self.socket
        if sock is not None and fcntl is not None and _FIONREAD is not None:
            try:
                buf = array.array('i', [0])
                fcntl.ioctl(sock.fileno(), _FIONREAD, buf)
                kernel = int(buf[0])
            except (OSError, ValueError):
                kernel = 0
        return kernel + len(self._buf)

    def _peek(self, n: int) -> bytes:
        """Return the first ``n`` buffered bytes without consuming them."""
        return bytes(self._buf[:n])

    def _consume(self, n: int) -> bytes:
        """Remove and return the first ``n`` buffered bytes."""
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def recv_exact(self, size: int) -> Optional[bytes]:
        """Return exactly ``size`` bytes, or None on timeout/close.

        Retained for callers/tests that want a blocking fixed-size read. Uses
        the persistent buffer, so a timeout never loses bytes.
        """
        r = self._ensure(size)
        if r is not True:
            return None
        return self._consume(size)

    # ------------------------------------------------------------------ #
    # Framing
    # ------------------------------------------------------------------ #
    def resync_to_preamble(self) -> bool:
        """Realign the buffer so it begins at the next valid preamble.

        Discards only the bytes *before* the preamble (confirmed garbage);
        everything from the preamble onward is preserved. Returns True once the
        buffer starts with a preamble, or False if the connection closed first.
        """
        self.resync_count += 1
        self.logger.info(
            f"Lost sync - searching for next preamble "
            f"(resync #{self.resync_count})...")

        while True:
            idx = self._buf.find(PREAMBLE)
            if idx >= 0:
                if idx > 0:
                    del self._buf[:idx]
                    self.bytes_discarded += idx
                self.logger.info(
                    f"Resync successful - found preamble "
                    f"(discarded {idx} bytes)")
                return True

            # No (complete) preamble yet. Keep the last 15 bytes in case a
            # preamble straddles the chunk boundary; drop the rest as garbage.
            keep = len(PREAMBLE) - 1
            if len(self._buf) > keep:
                dropped = len(self._buf) - keep
                del self._buf[:dropped]
                self.bytes_discarded += dropped

            r = self._recv_more()
            if r == 'closed':
                self.logger.warning("Connection closed during resync")
                return False
            # 'timeout' or 'data': loop again (buffer preserved either way)

    def read_ping(self) -> Optional[Tuple[DxHeader, DxData]]:
        """Read one complete ping (header + data) from the stream.

        Two-step protocol: a 20-byte DxHeader (16-byte preamble + 4-byte data
        length) followed by ``data_count`` bytes of DxData.

        Returns:
            (DxHeader, DxData) on success. None on a timeout (no complete frame
            yet - buffer preserved, call again) or when the connection drops.
        """
        if not self.connected:
            return None

        # Step 1: header (20 bytes).
        r = self._ensure(DxHeader.SIZE)
        if r is not True:
            return None  # timeout (retry) or closed - buffer preserved

        header = DxHeader.from_bytes(self._peek(DxHeader.SIZE))

        # Validate framing; resync if the preamble or length is implausible.
        if not header.is_valid() or not self._data_count_ok(header.data_count):
            if not header.is_valid():
                self.logger.warning(
                    f"Invalid preamble: {header.preamble[:8].hex()}... "
                    "(expected PING...)")
            else:
                self.logger.warning(
                    f"Implausible data_count={header.data_count}; resyncing")

            if not self.resync_to_preamble():
                return None
            # Buffer now starts at a preamble; re-read the header.
            r = self._ensure(DxHeader.SIZE)
            if r is not True:
                return None
            header = DxHeader.from_bytes(self._peek(DxHeader.SIZE))
            if not header.is_valid() or not self._data_count_ok(
                    header.data_count):
                # This preamble is followed by garbage length: drop the first
                # byte so the next resync locks onto a later preamble instead
                # of finding this same false one again.
                del self._buf[:1]
                self.bytes_discarded += 1
                return None

        # Step 2: full frame (header + payload).
        total = DxHeader.SIZE + header.data_count
        r = self._ensure(total)
        if r is not True:
            return None  # wait for the rest; header stays buffered

        frame = self._consume(total)
        data_bytes = frame[DxHeader.SIZE:]

        try:
            data = DxData.from_bytes(data_bytes)
        except Exception as e:
            # The frame was the length the header advertised, so the stream is
            # still aligned to the next preamble - just skip this bad ping.
            self.logger.error(f"Failed to parse data: {e}")
            return None

        self.logger.debug(f"Successfully parsed ping {data.ping_id}")
        return (header, data)

    def _data_count_ok(self, data_count: int) -> bool:
        """A payload must fit the fixed header and stay within a sane bound."""
        return DxData.HEADER_SIZE <= data_count <= MAX_DATA_COUNT

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
