#!/usr/bin/env python3
"""
3DSS-DX TCP Stream Data Structures

Python implementation of the structures defined in pingdsp-3dss.hpp
for parsing real-time 3DSS-DX sonar data over TCP.

Based on: 3DSS-DX Structure API v0.6 (2016-11-09)
Author: PingDSP Inc.
"""

import math
import struct
import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple


# Constants from C++ API (pingdsp-3dss.hpp v0.6)
# Expected preamble: 16-byte unique identifier
DX_PREAMBLE = bytes([0x50, 0x49, 0x4e, 0x47,  # "PING"
                     0x27, 0x2b, 0x3a, 0xd8,
                     0x74, 0x2a, 0x1c, 0x33,
                     0xe9, 0xb0, 0x73, 0xb1])
DX_TCP_PORT = 23848       # 3DSS-DX data stream TCP port (kTcpPort; UM002 §12)


@dataclass
class DxHeader:
    """
    20-byte header structure sent before each data packet.
    
    Structure:
    - 16 bytes: preamble (unique identifier)
    - 4 bytes: data_count (length of DxData to follow)
    
    Note: The original API incorrectly documented 3 reserved uint32 fields.
    Actual structure is 16-byte preamble + 4-byte data_count = 20 bytes total.
    """
    preamble: bytes        # uint8_t[16] - Should match DX_PREAMBLE
    data_count: int        # uint32_t - Length of DxData to follow in bytes
    
    SIZE = 20  # Total size in bytes (16 + 4)
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'DxHeader':
        """Parse DxHeader from 20-byte buffer."""
        if len(data) < cls.SIZE:
            raise ValueError(f"Buffer too small: expected {cls.SIZE}, got {len(data)}")
        
        preamble = data[:16]
        data_count = struct.unpack('<I', data[16:20])[0]
        
        return cls(
            preamble=preamble,
            data_count=data_count
        )
    
    def is_valid(self) -> bool:
        """Check if preamble matches expected value."""
        return self.preamble == DX_PREAMBLE


@dataclass
class Timestamp:
    """16-byte timestamp structure."""
    seconds: int       # uint64_t - seconds since epoch
    nanoseconds: int   # uint32_t - nanosecond remainder
    flags: int         # uint32_t - status flags
    
    SIZE = 16
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'Timestamp':
        """Parse Timestamp from 16-byte buffer."""
        if len(data) < cls.SIZE:
            raise ValueError(f"Buffer too small: expected {cls.SIZE}, got {len(data)}")
        
        seconds, nanoseconds, flags = struct.unpack('<QII', data[:cls.SIZE])
        return cls(seconds=seconds, nanoseconds=nanoseconds, flags=flags)


@dataclass
class SoundVelocity:
    """8-byte sound velocity structure."""
    bulk: float  # Bulk water column velocity (m/s)
    face: float  # Face velocity at transducer (m/s)
    
    SIZE = 8
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'SoundVelocity':
        """Parse SoundVelocity from 8-byte buffer."""
        if len(data) < cls.SIZE:
            raise ValueError(f"Buffer too small: expected {cls.SIZE}, got {len(data)}")
        
        bulk, face = struct.unpack('<ff', data[:cls.SIZE])
        return cls(bulk=bulk, face=face)


@dataclass
class Gain:
    """12-byte gain structure."""
    constant: float     # Constant gain (dB)
    linear: float       # Linear gain (dB/m)
    logarithmic: float  # Logarithmic gain (dB/log10(m))
    
    SIZE = 12
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'Gain':
        """Parse Gain from 12-byte buffer."""
        if len(data) < cls.SIZE:
            raise ValueError(f"Buffer too small: expected {cls.SIZE}, got {len(data)}")
        
        constant, linear, logarithmic = struct.unpack('<fff', data[:cls.SIZE])
        return cls(constant=constant, linear=linear, logarithmic=logarithmic)


@dataclass
class SidescanSettings:
    """128-byte sidescan settings structure."""
    mode: str                  # 32 bytes - mode name
    incoherent_method: str     # 32 bytes - incoherent method
    coherent_beams: str        # 64 bytes - beam list
    
    SIZE = 128
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'SidescanSettings':
        """Parse SidescanSettings from 128-byte buffer."""
        if len(data) < cls.SIZE:
            raise ValueError(f"Buffer too small: expected {cls.SIZE}, got {len(data)}")
        
        mode = data[0:32].split(b'\x00')[0].decode('utf-8', errors='ignore')
        incoherent_method = data[32:64].split(b'\x00')[0].decode('utf-8', errors='ignore')
        coherent_beams = data[64:128].split(b'\x00')[0].decode('utf-8', errors='ignore')
        
        return cls(mode=mode, incoherent_method=incoherent_method, coherent_beams=coherent_beams)


@dataclass
class Sidescan3DSettings:
    """16-byte sidescan 3D settings structure."""
    smoothing: int       # Number of samples for smoothing
    tolerance: float     # Tolerance for non-plane wave arrivals
    threshold: float     # Signal level threshold (dB re FS)
    number_of_angles: int  # Number of angles to compute
    
    SIZE = 16
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'Sidescan3DSettings':
        """Parse Sidescan3DSettings from 16-byte buffer."""
        if len(data) < cls.SIZE:
            raise ValueError(f"Buffer too small: expected {cls.SIZE}, got {len(data)}")
        
        smoothing, tolerance, threshold, number_of_angles = struct.unpack('<iffi', data[:cls.SIZE])
        return cls(smoothing=smoothing, tolerance=tolerance, threshold=threshold, 
                   number_of_angles=number_of_angles)


@dataclass
class TransmitSettings:
    """72-byte transmit settings structure."""
    angle: float        # Transmit beam angle (degrees)
    power: int          # Transmit power (percentage)
    beamwidth: str      # 32 bytes - beamwidth name
    pulse: str          # 32 bytes - pulse name
    
    SIZE = 72
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'TransmitSettings':
        """Parse TransmitSettings from 72-byte buffer."""
        if len(data) < cls.SIZE:
            raise ValueError(f"Buffer too small: expected {cls.SIZE}, got {len(data)}")
        
        angle, power = struct.unpack('<fI', data[0:8])
        beamwidth = data[8:40].split(b'\x00')[0].decode('utf-8', errors='ignore')
        pulse = data[40:72].split(b'\x00')[0].decode('utf-8', errors='ignore')
        
        return cls(angle=angle, power=power, beamwidth=beamwidth, pulse=pulse)


@dataclass
class TriggerSettings:
    """40-byte trigger settings structure."""
    source: str                    # 32 bytes - trigger source name
    continuous_duty_cycle: float   # Duty cycle for continuous mode
    reserved: float                # Reserved (likely ping rate)
    
    SIZE = 40
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'TriggerSettings':
        """Parse TriggerSettings from 40-byte buffer."""
        if len(data) < cls.SIZE:
            raise ValueError(f"Buffer too small: expected {cls.SIZE}, got {len(data)}")
        
        source = data[0:32].split(b'\x00')[0].decode('utf-8', errors='ignore')
        continuous_duty_cycle, reserved = struct.unpack('<ff', data[32:40])
        
        return cls(source=source, continuous_duty_cycle=continuous_duty_cycle, reserved=reserved)


@dataclass
class DxParameters:
    """576-byte sonar parameters structure."""
    range_m: float
    trigger: TriggerSettings
    sound_velocity: SoundVelocity
    port_gain: Gain
    starboard_gain: Gain
    port_sidescan: SidescanSettings
    starboard_sidescan: SidescanSettings
    port_sidescan3d: Sidescan3DSettings
    starboard_sidescan3d: Sidescan3DSettings
    port_transmit: TransmitSettings
    starboard_transmit: TransmitSettings
    reserved: bytes  # 68 bytes
    
    SIZE = 576
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'DxParameters':
        """Parse DxParameters from 576-byte buffer."""
        if len(data) < cls.SIZE:
            raise ValueError(f"Buffer too small: expected {cls.SIZE}, got {len(data)}")
        
        offset = 0
        
        # Range (4 bytes)
        range_m = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
        
        # TriggerSettings (40 bytes)
        trigger = TriggerSettings.from_bytes(data[offset:offset+TriggerSettings.SIZE])
        offset += TriggerSettings.SIZE
        
        # SoundVelocity (8 bytes)
        sound_velocity = SoundVelocity.from_bytes(data[offset:offset+SoundVelocity.SIZE])
        offset += SoundVelocity.SIZE
        
        # Port Gain (12 bytes)
        port_gain = Gain.from_bytes(data[offset:offset+Gain.SIZE])
        offset += Gain.SIZE
        
        # Starboard Gain (12 bytes)
        starboard_gain = Gain.from_bytes(data[offset:offset+Gain.SIZE])
        offset += Gain.SIZE
        
        # Port SidescanSettings (128 bytes)
        port_sidescan = SidescanSettings.from_bytes(data[offset:offset+SidescanSettings.SIZE])
        offset += SidescanSettings.SIZE
        
        # Starboard SidescanSettings (128 bytes)
        starboard_sidescan = SidescanSettings.from_bytes(data[offset:offset+SidescanSettings.SIZE])
        offset += SidescanSettings.SIZE
        
        # Port Sidescan3DSettings (16 bytes)
        port_sidescan3d = Sidescan3DSettings.from_bytes(
            data[offset:offset+Sidescan3DSettings.SIZE])
        offset += Sidescan3DSettings.SIZE
        
        # Starboard Sidescan3DSettings (16 bytes)
        starboard_sidescan3d = Sidescan3DSettings.from_bytes(
            data[offset:offset+Sidescan3DSettings.SIZE])
        offset += Sidescan3DSettings.SIZE
        
        # Port TransmitSettings (72 bytes)
        port_transmit = TransmitSettings.from_bytes(data[offset:offset+TransmitSettings.SIZE])
        offset += TransmitSettings.SIZE
        
        # Starboard TransmitSettings (72 bytes)
        starboard_transmit = TransmitSettings.from_bytes(data[offset:offset+TransmitSettings.SIZE])
        offset += TransmitSettings.SIZE
        
        # Reserved (68 bytes)
        reserved = data[offset:offset+68]
        
        return cls(
            range_m=range_m,
            trigger=trigger,
            sound_velocity=sound_velocity,
            port_gain=port_gain,
            starboard_gain=starboard_gain,
            port_sidescan=port_sidescan,
            starboard_sidescan=starboard_sidescan,
            port_sidescan3d=port_sidescan3d,
            starboard_sidescan3d=starboard_sidescan3d,
            port_transmit=port_transmit,
            starboard_transmit=starboard_transmit,
            reserved=reserved
        )


@dataclass
class DxSystemInfo:
    """128-byte system info structure."""
    sonar_id: str                          # 32 bytes
    acoustic_frequency: float              # Hz
    sample_rate: float                     # Hz
    maximum_ping_rate: float               # Hz
    port_sidescan_range_resolution: float  # meters
    starboard_sidescan_range_resolution: float  # meters
    port_sidescan3d_range_resolution: float     # meters
    starboard_sidescan3d_range_resolution: float  # meters
    port_transducer_angle: float           # degrees
    starboard_transducer_angle: float      # degrees
    reserved: bytes                        # 60 bytes
    
    SIZE = 128
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'DxSystemInfo':
        """Parse DxSystemInfo from 128-byte buffer."""
        if len(data) < cls.SIZE:
            raise ValueError(f"Buffer too small: expected {cls.SIZE}, got {len(data)}")
        
        sonar_id = data[0:32].split(b'\x00')[0].decode('utf-8', errors='ignore')
        values = struct.unpack('<10f', data[32:72])
        reserved = data[72:128]
        
        return cls(
            sonar_id=sonar_id,
            acoustic_frequency=values[0],
            sample_rate=values[1],
            maximum_ping_rate=values[2],
            port_sidescan_range_resolution=values[3],
            starboard_sidescan_range_resolution=values[4],
            port_sidescan3d_range_resolution=values[5],
            starboard_sidescan3d_range_resolution=values[6],
            port_transducer_angle=values[7],
            starboard_transducer_angle=values[8],
            reserved=reserved
        )


@dataclass
class BathymetryPoint:
    """
    20-byte bathymetry point structure.
    
    Generated from bottom-tracked and binned sidescan-3D data.
    Range and angle define position in polar coordinates.
    """
    range_m: float         # Range in meters
    angle_rad: float       # Angle in radians (downward angles are negative)
    amplitude: float       # Amplitude/intensity value
    reserved1: float       # Reserved for future use (quality factor)
    reserved2: float       # Reserved for future use
    
    SIZE = 20  # 5 floats × 4 bytes
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'BathymetryPoint':
        """Parse BathymetryPoint from 20-byte buffer."""
        if len(data) < cls.SIZE:
            raise ValueError(f"Buffer too small: expected {cls.SIZE}, got {len(data)}")
        
        values = struct.unpack('<5f', data[:cls.SIZE])
        return cls(
            range_m=values[0],
            angle_rad=values[1],  # Already in radians per API spec
            amplitude=values[2],
            reserved1=values[3],
            reserved2=values[4]
        )
    
    def to_xyz(self, is_port: bool = True,
               transducer_tilt_deg: float = 30.0) -> Tuple[float, float, float]:
        """
        Convert range/angle to XYZ coordinates in sonar frame.
        
        For dual-head side-scan sonar (port + starboard transducers):
        - X: forward (boat/survey direction)
        - Y: across-track (port=+Y, starboard=-Y)
        - Z: up (depth=-Z)
        
        Each transducer is tilted downward from horizontal:
        - transducer_tilt_deg: downward tilt angle (e.g., 30°)
        - angle_rad: beam angle relative to transducer axis
        - Negative angles = looking more downward from tilt axis
        
        Args:
            is_port: True for port side, False for starboard
            transducer_tilt_deg: Transducer downward tilt angle in degrees
        
        Returns:
            (x, y, z) tuple in meters
        """
        import math
        
        # Convert tilt to radians
        tilt_rad = math.radians(transducer_tilt_deg)
        
        # Total angle from horizontal = tilt + beam angle
        # Positive tilt = downward, negative beam angles = more downward
        total_angle = tilt_rad + self.angle_rad
        
        # Horizontal component (across-track)
        horizontal = self.range_m * math.cos(total_angle)
        
        # Vertical component (depth, negative = down)
        depth = self.range_m * math.sin(total_angle)
        
        # Port swath: positive Y, Starboard swath: negative Y
        if is_port:
            y = horizontal  # Port = positive Y (left)
        else:
            y = -horizontal  # Starboard = negative Y (right)
        
        x = 0.0  # No along-track resolution per ping
        z = depth
        
        return (x, y, z)


@dataclass
class SidescanPoint3D:
    """
    16-byte sidescan-3D point (vendor ``common::Sidescan3DPoint``).

    Polar like BathymetryPoint: range + angle from the transducer MRA
    (negative = downward) + amplitude. The fourth float is documented as
    "reserved for SNR or quality"; on real data it tracks 20*log10(amplitude)
    with a 5-95 % spread of 14-54 dB, i.e. it is the per-point SNR in dB.

    These are the raw CAATI angle solutions for every range step (several per
    range, water column included). The BathymetryPoint section is the
    bottom-tracked, binned subset of these.
    """
    range_m: float     # meters
    angle_rad: float   # radians, negative = downward from MRA
    amplitude: float   # after gain + 3D processing
    snr_db: float      # vendor "reserved"; SNR in dB on real data

    SIZE = 16  # 4 floats × 4 bytes

    @classmethod
    def from_bytes(cls, data: bytes) -> 'SidescanPoint3D':
        """Parse SidescanPoint3D from 16-byte buffer."""
        if len(data) < cls.SIZE:
            raise ValueError(f"Buffer too small: expected {cls.SIZE}, got {len(data)}")

        r, a, amp, snr = struct.unpack('<4f', data[:cls.SIZE])
        return cls(range_m=r, angle_rad=a, amplitude=amp, snr_db=snr)


def polar_to_sonar_xyz(ranges: np.ndarray, angles: np.ndarray, n_port: int,
                       tilt_port_deg: float, tilt_stbd_deg: float) -> np.ndarray:
    """Range/angle (port rows first, then starboard) -> Nx3 sonar-frame xyz.

    Same geometry as BathymetryPoint.to_xyz: total angle = mounting tilt +
    beam angle; x (along-track) = 0; port +y, starboard -y; z = depth
    (negative down).
    """
    n = ranges.shape[0]
    tilt = np.empty(n, dtype=np.float32)
    tilt[:n_port] = np.radians(tilt_port_deg)
    tilt[n_port:] = np.radians(tilt_stbd_deg)
    total = tilt + angles
    horizontal = ranges * np.cos(total)
    xyz = np.zeros((n, 3), dtype=np.float32)
    xyz[:n_port, 1] = horizontal[:n_port]
    xyz[n_port:, 1] = -horizontal[n_port:]
    xyz[:, 2] = ranges * np.sin(total)
    return xyz


# Hard physical sanity bound on ranges in any point section; corrupt frames
# can carry ~1e19. Not the user-tunable operating range (pointcloud_filter).
SANE_MAX_RANGE_M = 5000.0


@dataclass
class DxData:
    """
    872-byte header + variable-length data structure following DxHeader.
    
    Contains ping metadata, sonar settings, system info, and offsets/counts 
    for variable-length data sections.
    """
    
    # Ping identification (8 bytes)
    ping_id: int               # uint64_t - ping number
    
    # Timestamps (32 bytes)
    time: Timestamp            # Time trigger occurred
    time_range_zero: Timestamp # Zero range time
    
    # Sonar configuration (576 + 128 = 704 bytes)
    parameters: DxParameters   # All sonar settings
    system_info: DxSystemInfo  # System information
    
    # Data section offsets and counts (128 bytes = 32 × uint32_t)
    ascii_sentence_offset: int
    ascii_sentence_count: int
    
    port_sidescan_offset: int
    port_sidescan_count: int
    
    starboard_sidescan_offset: int
    starboard_sidescan_count: int
    
    port_sidescan3d_offset: int
    port_sidescan3d_count: int
    
    starboard_sidescan3d_offset: int
    starboard_sidescan3d_count: int
    
    port_bathymetry_offset: int
    port_bathymetry_count: int
    
    starboard_bathymetry_offset: int
    starboard_bathymetry_count: int
    
    recorded_filename_offset: int
    recorded_version_offset: int
    
    reserved: List[int]  # 16 × uint32_t reserved fields
    
    # Raw data buffer for extracting variable sections
    _raw_data: bytes
    
    HEADER_SIZE = 872  # 8 + 16 + 16 + 576 + 128 + 128
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'DxData':
        """Parse DxData from complete buffer."""
        if len(data) < cls.HEADER_SIZE:
            raise ValueError(f"Buffer too small: expected >={cls.HEADER_SIZE}, got {len(data)}")
        
        offset = 0
        
        # Ping ID (8 bytes)
        ping_id = struct.unpack('<Q', data[offset:offset+8])[0]
        offset += 8
        
        # Time (16 bytes)
        time = Timestamp.from_bytes(data[offset:offset+Timestamp.SIZE])
        offset += Timestamp.SIZE
        
        # Time range zero (16 bytes)
        time_range_zero = Timestamp.from_bytes(data[offset:offset+Timestamp.SIZE])
        offset += Timestamp.SIZE
        
        # Parameters (576 bytes)
        parameters = DxParameters.from_bytes(data[offset:offset+DxParameters.SIZE])
        offset += DxParameters.SIZE
        
        # System info (128 bytes)
        system_info = DxSystemInfo.from_bytes(data[offset:offset+DxSystemInfo.SIZE])
        offset += DxSystemInfo.SIZE
        
        # Offsets and counts (32 × uint32_t = 128 bytes)
        offsets_counts = struct.unpack('<32I', data[offset:offset+128])
        offset += 128

        # --- Sanity-clamp variable-section (offset, count) pairs ----------
        # A misaligned/corrupt frame still parses here (the 872-byte header is
        # fixed-size), but may carry garbage offsets/counts. Every getter loops
        # range(count), so a bogus count (e.g. ~3e9) spins in pure Python
        # effectively forever, holding the GIL, stalling the reader thread and
        # back-pressuring the entire TCP stream. Reject any (offset, count)
        # that cannot physically fit in the received buffer for its element
        # stride by zeroing the count so the getter returns empty.
        buf_len = len(data)

        def _sane(off_idx, cnt_idx, stride):
            off = offsets_counts[off_idx]
            cnt = offsets_counts[cnt_idx]
            if cnt <= 0 or off <= 0 or off >= buf_len:
                return off, 0
            if cnt > buf_len or off + cnt * stride > buf_len:
                return off, 0
            return off, cnt

        ascii_off, ascii_cnt = _sane(0, 1, 280)         # AsciiSentence stride
        p_ss_off, p_ss_cnt = _sane(2, 3, 8)             # SidescanPoint stride
        s_ss_off, s_ss_cnt = _sane(4, 5, 8)
        p_ss3_off, p_ss3_cnt = _sane(6, 7, SidescanPoint3D.SIZE)
        s_ss3_off, s_ss3_cnt = _sane(8, 9, SidescanPoint3D.SIZE)
        p_bat_off, p_bat_cnt = _sane(10, 11, BathymetryPoint.SIZE)
        s_bat_off, s_bat_cnt = _sane(12, 13, BathymetryPoint.SIZE)

        return cls(
            ping_id=ping_id,
            time=time,
            time_range_zero=time_range_zero,
            parameters=parameters,
            system_info=system_info,
            ascii_sentence_offset=ascii_off,
            ascii_sentence_count=ascii_cnt,
            port_sidescan_offset=p_ss_off,
            port_sidescan_count=p_ss_cnt,
            starboard_sidescan_offset=s_ss_off,
            starboard_sidescan_count=s_ss_cnt,
            port_sidescan3d_offset=p_ss3_off,
            port_sidescan3d_count=p_ss3_cnt,
            starboard_sidescan3d_offset=s_ss3_off,
            starboard_sidescan3d_count=s_ss3_cnt,
            port_bathymetry_offset=p_bat_off,
            port_bathymetry_count=p_bat_cnt,
            starboard_bathymetry_offset=s_bat_off,
            starboard_bathymetry_count=s_bat_cnt,
            recorded_filename_offset=offsets_counts[14],
            recorded_version_offset=offsets_counts[15],
            reserved=list(offsets_counts[16:32]),
            _raw_data=data
        )
    
    def get_ascii_sentences(self) -> str:
        """Extract ASCII sentences (NMEA, TSS1, etc.)."""
        if self.ascii_sentence_offset and self.ascii_sentence_count:
            # ASCII sentences are stored as AsciiSentence structures
            # Each is 280 bytes (16 + 256 + 4 + 4)
            sentences = []
            offset = self.ascii_sentence_offset
            for _ in range(self.ascii_sentence_count):
                # Skip timestamp (16 bytes)
                # Read sentence (256 bytes, null-terminated)
                sentence_bytes = self._raw_data[offset+16:offset+16+256]
                sentence = sentence_bytes.split(b'\x00')[0].decode('utf-8', errors='ignore')
                if sentence:
                    sentences.append(sentence)
                offset += 280  # sizeof(AsciiSentence)
            return '\n'.join(sentences)
        return ""
    
    def get_port_bathymetry(self) -> List[BathymetryPoint]:
        """Extract port side bathymetry points."""
        if not self.port_bathymetry_offset or not self.port_bathymetry_count:
            return []
        
        points = []
        offset = self.port_bathymetry_offset
        for _ in range(self.port_bathymetry_count):
            point = BathymetryPoint.from_bytes(self._raw_data[offset:offset+BathymetryPoint.SIZE])
            points.append(point)
            offset += BathymetryPoint.SIZE
        
        return points
    
    def get_starboard_bathymetry(self) -> List[BathymetryPoint]:
        """Extract starboard side bathymetry points."""
        if not self.starboard_bathymetry_offset or not self.starboard_bathymetry_count:
            return []
        
        points = []
        offset = self.starboard_bathymetry_offset
        for _ in range(self.starboard_bathymetry_count):
            point = BathymetryPoint.from_bytes(self._raw_data[offset:offset+BathymetryPoint.SIZE])
            points.append(point)
            offset += BathymetryPoint.SIZE
        
        return points
    
    def _sidescan_points(self, offset: int, count: int) -> np.ndarray:
        """Return a (count, 2) float32 view of SidescanPoint structs: [range_m, amplitude]."""
        if not offset or not count:
            return np.empty((0, 2), dtype=np.float32)
        return np.frombuffer(self._raw_data, dtype='<f4', count=2 * count,
                             offset=offset).reshape(count, 2)

    def get_port_sidescan(self) -> np.ndarray:
        """Extract port sidescan amplitudes as a float32 array."""
        return np.ascontiguousarray(
            self._sidescan_points(self.port_sidescan_offset,
                                  self.port_sidescan_count)[:, 1])

    def get_starboard_sidescan(self) -> np.ndarray:
        """Extract starboard sidescan amplitudes as a float32 array."""
        return np.ascontiguousarray(
            self._sidescan_points(self.starboard_sidescan_offset,
                                  self.starboard_sidescan_count)[:, 1])

    def get_port_sidescan_ranges(self) -> np.ndarray:
        """Per-sample slant range (m) the head attached to each port sample."""
        return np.ascontiguousarray(
            self._sidescan_points(self.port_sidescan_offset,
                                  self.port_sidescan_count)[:, 0])

    def get_starboard_sidescan_ranges(self) -> np.ndarray:
        """Per-sample slant range (m) for each starboard sample."""
        return np.ascontiguousarray(
            self._sidescan_points(self.starboard_sidescan_offset,
                                  self.starboard_sidescan_count)[:, 0])

    def sidescan_bin_spacing_m(self) -> float:
        """Return the expected metres per sidescan sample: c_bulk / (2 fs).

        The head sends one SidescanPoint per receive sample starting at range 0,
        so consumers reconstruct range as ``i * c / (2 * sample_rate)``. (The
        ``*_sidescan_range_resolution`` field is the pulse range resolution,
        not this spacing.) Returns 0.0 if either input is missing.
        """
        c = float(self.parameters.sound_velocity.bulk)
        fs = float(self.system_info.sample_rate)
        if c <= 0.0 or fs <= 0.0:
            return 0.0
        return c / (2.0 * fs)

    def check_sidescan_grid(self, rel_tol: float = 0.01) -> List[str]:
        """Verify the per-sample ranges match the i*c/(2fs) grid consumers assume.

        Downstream code (bags, exporters) discards the per-sample ranges and
        rebuilds them from ``sample_rate`` + ``sound_velocity_bulk``. That is
        only valid while the head emits every receive sample from range 0. If a
        firmware/mode ever decimates or offsets the 2D sidescan (the 3D sidescan
        already does), this flags it.

        Checks per side (only where samples exist):
          * first sample range within one bin of 0
          * mean step ``(r[-1] - r[0]) / (N - 1)`` within ``rel_tol`` of c/(2fs)

        Returns a list of human-readable problems; empty list means OK.
        """
        expected = self.sidescan_bin_spacing_m()
        problems: List[str] = []
        if expected <= 0.0:
            return problems  # cannot evaluate without c and fs
        for side, off, cnt in (
                ('port', self.port_sidescan_offset, self.port_sidescan_count),
                ('starboard', self.starboard_sidescan_offset,
                 self.starboard_sidescan_count)):
            if cnt < 2:
                continue
            r = self._sidescan_points(off, cnt)[:, 0]
            r0 = float(r[0])
            step = (float(r[-1]) - r0) / float(cnt - 1)
            if abs(r0) > expected:
                problems.append(
                    f'{side} sidescan first sample at {r0:.4f} m, expected 0 '
                    f'(start offset)')
            if not math.isfinite(step) or abs(step - expected) > rel_tol * expected:
                problems.append(
                    f'{side} sidescan step {step:.5f} m/sample vs c/(2fs) '
                    f'{expected:.5f} m (N={cnt}, span {float(r[-1]):.2f} m)')
        return problems

    def frame_problems(self) -> List[str]:
        """Return reasons this frame's payload is not trustworthy ([] = clean).

        A frame can parse (the 872-byte header is fixed-size and the section
        offsets are clamped to the buffer) and still be garbage: when bytes go
        missing upstream - a pcap with capture loss, a replayer skipping, a
        link that dropped mid-frame - the next frame's bytes get spliced into
        this frame's body at the advertised length. The symptoms are concrete
        and cheap to test for, so a corrupt frame can be dropped whole instead
        of leaking 1e19-metre points and ``inf`` amplitudes into every topic.

        Checks:
          * no DX preamble inside the body (a spliced-in next frame)
          * sidescan sample grid is ``i*c/(2fs)`` (see check_sidescan_grid)
          * sidescan amplitudes finite
          * bathymetry / sidescan-3D ranges finite and within the range setting
            (with slack) and angles within +-pi
          * bathymetry quality counts are small integers, not reinterpreted
            float garbage
        """
        problems: List[str] = []
        body = self._raw_data
        idx = body.find(DX_PREAMBLE, 1)
        if idx >= 0:
            problems.append(f'DX preamble inside body at byte {idx} (spliced frame)')
            return problems  # everything after idx is another frame; no point going on

        problems.extend(self.check_sidescan_grid())

        for side, off, cnt in (
                ('port', self.port_sidescan_offset, self.port_sidescan_count),
                ('starboard', self.starboard_sidescan_offset,
                 self.starboard_sidescan_count)):
            if cnt:
                amp = self._sidescan_points(off, cnt)[:, 1]
                if not bool(np.isfinite(amp).all()):
                    problems.append(f'{side} sidescan has non-finite amplitudes')

        range_setting = float(self.parameters.range_m)
        # Points can legitimately exceed the range setting slightly (the head
        # keeps the tail of the receive window); garbage is orders off.
        max_r = max(1.5 * range_setting, 10.0) if range_setting > 0 else SANE_MAX_RANGE_M
        for name, off, cnt, raw in (
                ('port bathymetry', self.port_bathymetry_offset,
                 self.port_bathymetry_count, self._bathymetry_raw),
                ('starboard bathymetry', self.starboard_bathymetry_offset,
                 self.starboard_bathymetry_count, self._bathymetry_raw),
                ('port sidescan3d', self.port_sidescan3d_offset,
                 self.port_sidescan3d_count, self._sidescan3d_raw),
                ('starboard sidescan3d', self.starboard_sidescan3d_offset,
                 self.starboard_sidescan3d_count, self._sidescan3d_raw)):
            if not cnt:
                continue
            pts = raw(off, cnt)
            r, a = pts[:, 0], pts[:, 1]
            if not bool(np.isfinite(pts).all()):
                problems.append(f'{name} has non-finite values')
                continue
            if bool((r < 0).any()) or bool((r > max_r).any()):
                problems.append(
                    f'{name} range outside [0, {max_r:.0f}] m '
                    f'(min {float(r.min()):.2f}, max {float(r.max()):.3g})')
            if bool((np.abs(a) > math.pi).any()):
                problems.append(f'{name} angle beyond +-pi (max {float(np.abs(a).max()):.3g})')

        for side, off, cnt in (
                ('port', self.port_bathymetry_offset, self.port_bathymetry_count),
                ('starboard', self.starboard_bathymetry_offset,
                 self.starboard_bathymetry_count)):
            if cnt:
                q = self._bathymetry_quality(off, cnt)
                if float(q.max()) > 1.0e6:
                    problems.append(
                        f'{side} bathymetry quality count {float(q.max()):.3g} '
                        '(not a sample count)')
        return problems
    
    def get_all_bathymetry_xyz(self, transducer_tilt_deg: float = 30.0) -> np.ndarray:
        """
        Get all bathymetry points (port + starboard) as XYZ array.
        
        Args:
            transducer_tilt_deg: Transducer downward tilt angle in degrees
        
        Returns:
            Nx3 numpy array of (x, y, z) coordinates
        """
        port_points = self.get_port_bathymetry()
        starboard_points = self.get_starboard_bathymetry()
        
        n_port = len(port_points)
        n_stbd = len(starboard_points)
        n_total = n_port + n_stbd
        
        if n_total == 0:
            return np.empty((0, 3), dtype=np.float32)
        
        # Vectorized extraction of range and angle arrays
        port_data = np.array([(p.range_m, p.angle_rad) for p in port_points],
                             dtype=np.float32)
        stbd_data = np.array([(p.range_m, p.angle_rad) for p in starboard_points],
                             dtype=np.float32)
        
        # Combine into single arrays
        if n_port > 0 and n_stbd > 0:
            ranges = np.concatenate([port_data[:, 0], stbd_data[:, 0]])
            angles = np.concatenate([port_data[:, 1], stbd_data[:, 1]])
        elif n_port > 0:
            ranges = port_data[:, 0]
            angles = port_data[:, 1]
        else:
            ranges = stbd_data[:, 0]
            angles = stbd_data[:, 1]
        
        # Vectorized computation using same logic as to_xyz()
        tilt_rad = np.radians(transducer_tilt_deg)
        total_angles = tilt_rad + angles
        
        # Compute horizontal and depth components
        horizontal = ranges * np.cos(total_angles)
        depth = ranges * np.sin(total_angles)
        
        # Create XYZ array
        xyz = np.zeros((n_total, 3), dtype=np.float32)
        xyz[:, 0] = 0.0  # X (along-track)
        xyz[:n_port, 1] = horizontal[:n_port]   # Port: positive Y
        xyz[n_port:, 1] = -horizontal[n_port:]  # Starboard: negative Y
        xyz[:, 2] = depth  # Z (depth)

        # Reject corrupt points before they leave the parser. A desynced or
        # truncated frame can yield garbage ranges (~1e19) that later overflow
        # float32 arithmetic downstream. ranges/angles share xyz row order
        # (port rows first, then starboard), so the mask applies directly.
        # SANE_MAX_RANGE is a hard physical sanity bound, not the user-tunable
        # operating range (that filtering happens in pointcloud_filter).
        SANE_MAX_RANGE = 5000.0  # metres
        valid = (np.isfinite(ranges) & np.isfinite(angles)
                 & (ranges >= 0.0) & (ranges <= SANE_MAX_RANGE))
        if not bool(valid.all()):
            xyz = xyz[valid]

        return xyz

    def get_all_bathymetry_xyzi(
            self, transducer_tilt_deg: float = 30.0,
            tilt_port_deg: float = None,
            tilt_stbd_deg: float = None) -> np.ndarray:
        """All bathymetry as Nx4 (x, y, z, intensity), corrupt points removed.

        Unlike pairing get_all_bathymetry_xyz() with a separately gathered
        amplitude list, this filters xyz AND intensity with the *same* validity
        mask, so the columns always stay aligned -- a single corrupt point no
        longer causes a length mismatch that drops the whole ping.

        tilt_port_deg / tilt_stbd_deg allow a different tilt per side (the
        sonar reports the two mounting angles independently); either falling
        back to transducer_tilt_deg when None.
        """
        if tilt_port_deg is None:
            tilt_port_deg = transducer_tilt_deg
        if tilt_stbd_deg is None:
            tilt_stbd_deg = transducer_tilt_deg
        port_points = self.get_port_bathymetry()
        starboard_points = self.get_starboard_bathymetry()

        n_port = len(port_points)
        n_stbd = len(starboard_points)
        n_total = n_port + n_stbd

        if n_total == 0:
            return np.empty((0, 4), dtype=np.float32)

        rows = [(p.range_m, p.angle_rad, p.amplitude) for p in port_points]
        rows += [(p.range_m, p.angle_rad, p.amplitude)
                 for p in starboard_points]
        all_data = np.array(rows, dtype=np.float32)
        ranges = all_data[:, 0]
        angles = all_data[:, 1]
        amplitudes = all_data[:, 2]

        tilt_rad = np.empty(n_total, dtype=np.float32)
        tilt_rad[:n_port] = np.radians(tilt_port_deg)
        tilt_rad[n_port:] = np.radians(tilt_stbd_deg)
        total_angles = tilt_rad + angles
        horizontal = ranges * np.cos(total_angles)
        depth = ranges * np.sin(total_angles)

        xyzi = np.zeros((n_total, 4), dtype=np.float32)
        xyzi[:n_port, 1] = horizontal[:n_port]    # Port: positive Y
        xyzi[n_port:, 1] = -horizontal[n_port:]   # Starboard: negative Y
        xyzi[:, 2] = depth
        xyzi[:, 3] = np.nan_to_num(
            amplitudes, nan=0.0, posinf=0.0, neginf=0.0)

        SANE_MAX_RANGE = 5000.0  # metres
        valid = (np.isfinite(ranges) & np.isfinite(angles)
                 & (ranges >= 0.0) & (ranges <= SANE_MAX_RANGE))
        if not bool(valid.all()):
            xyzi = xyzi[valid]

        return xyzi

    # ----- vectorised point-section access -------------------------------

    def _bathymetry_raw(self, offset: int, count: int) -> np.ndarray:
        """(count, 5) float32 view of BathymetryPoint structs.

        Columns: range, angle, amplitude, reserved1, reserved2. ``reserved1``
        is actually a uint32 written into the float slot (reads as denormals);
        use :meth:`_bathymetry_quality` to get it as a number.
        """
        if not offset or not count:
            return np.empty((0, 5), dtype=np.float32)
        return np.frombuffer(self._raw_data, dtype='<f4', count=5 * count,
                             offset=offset).reshape(count, 5)

    def _bathymetry_quality(self, offset: int, count: int) -> np.ndarray:
        """Per-point ``reserved1`` of BathymetryPoint reinterpreted as uint32.

        On real data this is an integer 1..~200 (median ~9) that grows with
        range and is independent of amplitude - consistent with the number of
        sidescan-3D samples binned into the point, i.e. a quality/confidence
        count. Returned as float32 so it can ride in a PointCloud2 field.
        """
        if not offset or not count:
            return np.empty((0,), dtype=np.float32)
        u = np.frombuffer(self._raw_data, dtype='<u4', count=5 * count,
                          offset=offset).reshape(count, 5)[:, 3]
        return u.astype(np.float32)

    def get_all_bathymetry_xyziq(
            self, tilt_port_deg: float, tilt_stbd_deg: float) -> np.ndarray:
        """All bathymetry as Nx5 (x, y, z, intensity, quality), vectorised.

        Same geometry and corruption mask as :meth:`get_all_bathymetry_xyzi`,
        plus the per-point sample count from the vendor ``reserved1`` slot as
        ``quality``. Reads the sections with ``np.frombuffer`` instead of
        building a BathymetryPoint object per point.
        """
        port = self._bathymetry_raw(self.port_bathymetry_offset,
                                    self.port_bathymetry_count)
        stbd = self._bathymetry_raw(self.starboard_bathymetry_offset,
                                    self.starboard_bathymetry_count)
        n_port = port.shape[0]
        n_total = n_port + stbd.shape[0]
        if n_total == 0:
            return np.empty((0, 5), dtype=np.float32)
        raw = np.concatenate([port, stbd]) if stbd.shape[0] else port
        ranges = raw[:, 0]
        angles = raw[:, 1]
        quality = np.concatenate([
            self._bathymetry_quality(self.port_bathymetry_offset,
                                     self.port_bathymetry_count),
            self._bathymetry_quality(self.starboard_bathymetry_offset,
                                     self.starboard_bathymetry_count)])

        out = np.empty((n_total, 5), dtype=np.float32)
        out[:, :3] = polar_to_sonar_xyz(ranges, angles, n_port,
                                        tilt_port_deg, tilt_stbd_deg)
        out[:, 3] = np.nan_to_num(raw[:, 2], nan=0.0, posinf=0.0, neginf=0.0)
        out[:, 4] = quality

        valid = (np.isfinite(ranges) & np.isfinite(angles)
                 & (ranges >= 0.0) & (ranges <= SANE_MAX_RANGE_M))
        if not bool(valid.all()):
            out = out[valid]
        return out

    def _sidescan3d_raw(self, offset: int, count: int) -> np.ndarray:
        """(count, 4) float32 view of Sidescan3DPoint: range, angle, amp, snr."""
        if not offset or not count:
            return np.empty((0, 4), dtype=np.float32)
        return np.frombuffer(self._raw_data, dtype='<f4', count=4 * count,
                             offset=offset).reshape(count, 4)

    def get_port_sidescan3d(self) -> np.ndarray:
        """Port sidescan-3D points as (N, 4): range_m, angle_rad, amplitude, snr_db."""
        return np.ascontiguousarray(self._sidescan3d_raw(
            self.port_sidescan3d_offset, self.port_sidescan3d_count))

    def get_starboard_sidescan3d(self) -> np.ndarray:
        """Starboard sidescan-3D points as (N, 4): range_m, angle_rad, amplitude, snr_db."""
        return np.ascontiguousarray(self._sidescan3d_raw(
            self.starboard_sidescan3d_offset, self.starboard_sidescan3d_count))

    def get_all_sidescan3d_xyzis(
            self, tilt_port_deg: float, tilt_stbd_deg: float) -> np.ndarray:
        """All sidescan-3D points as Nx5 (x, y, z, intensity, snr_db).

        This is the full 3D point set the head computes - every range step,
        several angles per range, water column included - of which the
        bathymetry section is the bottom-tracked subset. Same sonar-frame
        geometry and corruption mask as the bathymetry cloud.
        """
        port = self._sidescan3d_raw(self.port_sidescan3d_offset,
                                    self.port_sidescan3d_count)
        stbd = self._sidescan3d_raw(self.starboard_sidescan3d_offset,
                                    self.starboard_sidescan3d_count)
        n_port = port.shape[0]
        n_total = n_port + stbd.shape[0]
        if n_total == 0:
            return np.empty((0, 5), dtype=np.float32)
        raw = np.concatenate([port, stbd]) if stbd.shape[0] else port
        ranges = raw[:, 0]
        angles = raw[:, 1]

        out = np.empty((n_total, 5), dtype=np.float32)
        out[:, :3] = polar_to_sonar_xyz(ranges, angles, n_port,
                                        tilt_port_deg, tilt_stbd_deg)
        out[:, 3] = np.nan_to_num(raw[:, 2], nan=0.0, posinf=0.0, neginf=0.0)
        out[:, 4] = np.nan_to_num(raw[:, 3], nan=0.0, posinf=0.0, neginf=0.0)

        valid = (np.isfinite(ranges) & np.isfinite(angles)
                 & (ranges >= 0.0) & (ranges <= SANE_MAX_RANGE_M))
        if not bool(valid.all()):
            out = out[valid]
        return out

    def settings_snapshot(self) -> dict:
        """Flat dict of every DxParameters/DxSystemInfo value for this ping.

        Keys match ``pingdsp_msg/SonarSettings`` field names so the driver can
        assign them directly and compare snapshots between pings to publish
        only on change.
        """
        p, si = self.parameters, self.system_info
        return {
            'sonar_id': si.sonar_id,
            'acoustic_frequency': float(si.acoustic_frequency),
            'sample_rate': float(si.sample_rate),
            'maximum_ping_rate': float(si.maximum_ping_rate),
            'port_transducer_angle': float(si.port_transducer_angle),
            'starboard_transducer_angle': float(si.starboard_transducer_angle),
            'range': float(p.range_m),
            'sidescan_bin_spacing': float(self.sidescan_bin_spacing_m()),
            'port_sidescan_range_resolution': float(si.port_sidescan_range_resolution),
            'starboard_sidescan_range_resolution': float(si.starboard_sidescan_range_resolution),
            'port_sidescan3d_range_resolution': float(si.port_sidescan3d_range_resolution),
            'starboard_sidescan3d_range_resolution':
                float(si.starboard_sidescan3d_range_resolution),
            'sound_velocity_bulk': float(p.sound_velocity.bulk),
            'sound_velocity_face': float(p.sound_velocity.face),
            'port_gain_constant': float(p.port_gain.constant),
            'port_gain_linear': float(p.port_gain.linear),
            'port_gain_logarithmic': float(p.port_gain.logarithmic),
            'starboard_gain_constant': float(p.starboard_gain.constant),
            'starboard_gain_linear': float(p.starboard_gain.linear),
            'starboard_gain_logarithmic': float(p.starboard_gain.logarithmic),
            'port_transmit_angle': float(p.port_transmit.angle),
            'port_transmit_power': int(p.port_transmit.power),
            'port_transmit_beamwidth': p.port_transmit.beamwidth,
            'port_transmit_pulse': p.port_transmit.pulse,
            'starboard_transmit_angle': float(p.starboard_transmit.angle),
            'starboard_transmit_power': int(p.starboard_transmit.power),
            'starboard_transmit_beamwidth': p.starboard_transmit.beamwidth,
            'starboard_transmit_pulse': p.starboard_transmit.pulse,
            'trigger_source': p.trigger.source,
            'trigger_continuous_duty_cycle': float(p.trigger.continuous_duty_cycle),
            'port_sidescan_mode': p.port_sidescan.mode,
            'port_sidescan_incoherent_method': p.port_sidescan.incoherent_method,
            'port_sidescan_coherent_beams': p.port_sidescan.coherent_beams,
            'starboard_sidescan_mode': p.starboard_sidescan.mode,
            'starboard_sidescan_incoherent_method': p.starboard_sidescan.incoherent_method,
            'starboard_sidescan_coherent_beams': p.starboard_sidescan.coherent_beams,
            'port_sidescan3d_smoothing': int(p.port_sidescan3d.smoothing),
            'port_sidescan3d_tolerance': float(p.port_sidescan3d.tolerance),
            'port_sidescan3d_threshold': float(p.port_sidescan3d.threshold),
            'port_sidescan3d_number_of_angles': int(p.port_sidescan3d.number_of_angles),
            'starboard_sidescan3d_smoothing': int(p.starboard_sidescan3d.smoothing),
            'starboard_sidescan3d_tolerance': float(p.starboard_sidescan3d.tolerance),
            'starboard_sidescan3d_threshold': float(p.starboard_sidescan3d.threshold),
            'starboard_sidescan3d_number_of_angles': int(p.starboard_sidescan3d.number_of_angles),
        }
    
    # Physically plausible mounting tilt magnitudes; anything outside this is a
    # zeroed/garbage field rather than a real angle.
    _SANE_TILT_RANGE = (1.0, 89.0)

    def reported_transducer_tilts(self, fallback_deg: float):
        """Per-side transducer tilt in this parser's sign convention.

        The sonar reports its housing mounting angles with **positive =
        downward** (vendor ``DxSystemInfo``), whereas the geometry here treats
        **negative = downward** (``total_angle = tilt + beam_angle``), so the
        reported values are negated. Using the reported angles keeps the
        solution correct if the two sides are mounted asymmetrically, which a
        single tilt parameter cannot represent.

        A side whose reported angle is missing or implausible falls back to
        ``fallback_deg``.

        Returns:
            (tilt_port_deg, tilt_stbd_deg)
        """
        lo, hi = self._SANE_TILT_RANGE
        out = []
        for reported in (self.system_info.port_transducer_angle,
                         self.system_info.starboard_transducer_angle):
            try:
                val = float(reported)
            except (TypeError, ValueError):
                out.append(fallback_deg)
                continue
            if math.isfinite(val) and lo <= abs(val) <= hi:
                out.append(-abs(val))
            else:
                out.append(fallback_deg)
        return out[0], out[1]

    def get_recorded_filename(self) -> str:
        """Extract recorded filename string."""
        if self.recorded_filename_offset:
            offset = self.recorded_filename_offset
            end = self._raw_data.find(b'\x00', offset)
            if end > offset:
                return self._raw_data[offset:end].decode('utf-8', errors='ignore')
        return ""
    
    def get_recorded_version(self) -> str:
        """Extract recorded version string."""
        if self.recorded_version_offset:
            offset = self.recorded_version_offset
            end = self._raw_data.find(b'\x00', offset)
            if end > offset:
                return self._raw_data[offset:end].decode('utf-8', errors='ignore')
        return ""
    
    # ===== Compatibility properties for old driver code =====
    
    @property
    def milliseconds_today(self) -> int:
        """Compatibility: Convert timestamp to milliseconds since midnight."""
        return int((self.time.seconds % 86400) * 1000 + (self.time.nanoseconds // 1_000_000))
    
    @property
    def ping_number(self) -> int:
        """Compatibility: ping_id alias."""
        return self.ping_id
    
    @property
    def port_bathy_count(self) -> int:
        """Compatibility: port bathymetry count."""
        return self.port_bathymetry_count
    
    @property
    def stbd_bathy_count(self) -> int:
        """Compatibility: starboard bathymetry count."""
        return self.starboard_bathymetry_count
    
    @property
    def sample_rate_hz(self) -> float:
        """Compatibility: sample rate from system info."""
        return float(self.system_info.sample_rate)
    
    @property
    def ping_rate_hz(self) -> float:
        """Compatibility: maximum ping rate from system info."""
        return float(self.system_info.maximum_ping_rate)
    
    def get_stbd_bathymetry(self) -> List[BathymetryPoint]:
        """Compatibility: alias for get_starboard_bathymetry."""
        return self.get_starboard_bathymetry()
    
    def get_stbd_sidescan(self) -> np.ndarray:
        """Compatibility: alias for get_starboard_sidescan."""
        return self.get_starboard_sidescan()
    
    def get_ascii_data(self) -> str:
        """Compatibility: alias for get_ascii_sentences."""
        return self.get_ascii_sentences()
