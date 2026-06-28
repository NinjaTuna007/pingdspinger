"""Build synthetic 3DSS-DX wire frames for tests.

The real sonar streams ``DxHeader`` (20 bytes) + ``DxData`` (>= 872-byte header
plus variable sections). Re-creating a valid frame here lets us exercise both
``dx_structures`` parsing (unit) and the live driver over a fake TCP server
(integration) without any captured data.

Layout produced (matches :mod:`pingdsp_driver.dx_structures`)::

    [20]  DxHeader: 16-byte preamble + uint32 data_count
    [872] DxData header: ping_id, 2x Timestamp, Parameters(576),
          SystemInfo(128), 32x uint32 offset/count
    [...] variable sections referenced by the offsets:
          ascii sentences (280 B each), port/stbd sidescan (8 B each),
          port/stbd bathymetry (20 B each)
"""

import struct

from pingdsp_driver.dx_structures import DX_PREAMBLE, DxData

_HEADER_SIZE = DxData.HEADER_SIZE  # 872
_ASCII_RECORD = 280
_SIDESCAN_RECORD = 8
_BATHY_RECORD = 20


def _build_parameters(range_m=50.0, sv_bulk=1500.0, sv_face=1500.0,
                      tx_angle=0.0, tx_power=80):
    """Return a 576-byte DxParameters blob with a few fields populated."""
    buf = bytearray(576)
    struct.pack_into('<f', buf, 0, range_m)
    # TriggerSettings (40) at offset 4: 32-byte source + 2 floats - leave zeros.
    # SoundVelocity (8) at offset 44.
    struct.pack_into('<ff', buf, 44, sv_bulk, sv_face)
    # Gains (port@52, stbd@64), sidescan settings, etc. - leave zeros.
    # Port TransmitSettings starts at 4+40+8+12+12+128+128+16+16 = 364.
    struct.pack_into('<fI', buf, 364, tx_angle, tx_power)
    return bytes(buf)


def _build_system_info(sonar_id='TEST-3DSS', sample_rate=50000.0,
                       ping_rate=10.0, port_angle=-30.0, stbd_angle=30.0):
    """Return a 128-byte DxSystemInfo blob."""
    buf = bytearray(128)
    sid = sonar_id.encode('utf-8')[:31]
    buf[0:len(sid)] = sid
    # 10 floats at offset 32: freq, sample_rate, max_ping_rate, 4x res,
    # port_angle, stbd_angle, spare.
    struct.pack_into('<10f', buf, 32,
                     400000.0, sample_rate, ping_rate,
                     0.05, 0.05, 0.05, 0.05,
                     port_angle, stbd_angle, 0.0)
    return bytes(buf)


def build_dx_data(ping_id=1, ascii_sentences=None,
                  port_sidescan=None, starboard_sidescan=None,
                  port_bathy=None, starboard_bathy=None,
                  sample_rate=50000.0, ping_rate=10.0, range_m=50.0):
    """Build the DxData blob (872-byte header + variable sections).

    Args:
        ascii_sentences: list[str] NMEA/TSS1 sentences.
        port_sidescan / starboard_sidescan: list[float] amplitudes.
        port_bathy / starboard_bathy: list[(range_m, angle_rad, amplitude)].

    Returns:
        bytes (the DxData payload, i.e. what follows the DxHeader).
    """
    ascii_sentences = ascii_sentences or []
    port_sidescan = port_sidescan or []
    starboard_sidescan = starboard_sidescan or []
    port_bathy = port_bathy or []
    starboard_bathy = starboard_bathy or []

    header = bytearray(_HEADER_SIZE)
    struct.pack_into('<Q', header, 0, ping_id)
    # time @8, time_range_zero @24 (Timestamp: Q seconds, I ns, I flags).
    struct.pack_into('<QII', header, 8, 1_700_000_000, 0, 0)
    struct.pack_into('<QII', header, 24, 1_700_000_000, 0, 0)
    header[40:40 + 576] = _build_parameters(range_m=range_m)
    header[616:616 + 128] = _build_system_info(
        sample_rate=sample_rate, ping_rate=ping_rate)

    # Variable sections appended after the fixed header.
    body = bytearray()
    cursor = _HEADER_SIZE

    def _section(payload):
        nonlocal cursor
        start = cursor
        body.extend(payload)
        cursor += len(payload)
        return start

    ascii_blob = bytearray()
    for sentence in ascii_sentences:
        rec = bytearray(_ASCII_RECORD)
        enc = sentence.encode('utf-8')[:255]
        rec[16:16 + len(enc)] = enc
        ascii_blob.extend(rec)
    ascii_off = _section(ascii_blob) if ascii_sentences else 0

    def _ss_blob(samples):
        blob = bytearray()
        for amp in samples:
            blob.extend(struct.pack('<ff', 0.0, float(amp)))
        return blob

    port_ss_off = _section(_ss_blob(port_sidescan)) if port_sidescan else 0
    stbd_ss_off = (_section(_ss_blob(starboard_sidescan))
                   if starboard_sidescan else 0)

    def _bathy_blob(points):
        blob = bytearray()
        for rng, ang, amp in points:
            blob.extend(struct.pack('<5f', rng, ang, amp, 0.0, 0.0))
        return blob

    port_bathy_off = _section(_bathy_blob(port_bathy)) if port_bathy else 0
    stbd_bathy_off = (_section(_bathy_blob(starboard_bathy))
                      if starboard_bathy else 0)

    offsets = [
        ascii_off, len(ascii_sentences),
        port_ss_off, len(port_sidescan),
        stbd_ss_off, len(starboard_sidescan),
        0, 0,  # port/stbd sidescan3d
        0, 0,
        port_bathy_off, len(port_bathy),
        stbd_bathy_off, len(starboard_bathy),
        0, 0,  # recorded filename/version offsets
    ] + [0] * 16
    struct.pack_into('<32I', header, 744, *offsets)

    return bytes(header) + bytes(body)


def build_dx_frame(**kwargs):
    """Build a full wire frame: DxHeader + DxData (ready to send over TCP)."""
    data = build_dx_data(**kwargs)
    return DX_PREAMBLE + struct.pack('<I', len(data)) + data
