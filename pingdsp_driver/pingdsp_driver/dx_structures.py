#!/usr/bin/env python3
"""
3DSS-DX TCP Stream Data Structures

Python implementation of the structures defined in pingdsp-3dss.hpp
for parsing real-time 3DSS-DX sonar data over TCP.

Based on: 3DSS-DX Structure API v0.6 (2016-11-09)
Author: PingDSP Inc.
"""

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
DX_TCP_PORT = 14001       # Default TCP port for 3DSS-DX


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
        port_sidescan3d = Sidescan3DSettings.from_bytes(data[offset:offset+Sidescan3DSettings.SIZE])
        offset += Sidescan3DSettings.SIZE
        
        # Starboard Sidescan3DSettings (16 bytes)
        starboard_sidescan3d = Sidescan3DSettings.from_bytes(data[offset:offset+Sidescan3DSettings.SIZE])
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
    
    def to_xyz(self, is_port: bool = True, transducer_tilt_deg: float = 30.0) -> Tuple[float, float, float]:
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
    12-byte 3D sidescan point structure.
    
    Each point contains XYZ position relative to the sensor.
    """
    x: float  # Across-track position (meters)
    y: float  # Along-track position (meters)
    z: float  # Depth (meters, negative down)
    
    SIZE = 12  # 3 floats × 4 bytes
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'SidescanPoint3D':
        """Parse SidescanPoint3D from 12-byte buffer."""
        if len(data) < cls.SIZE:
            raise ValueError(f"Buffer too small: expected {cls.SIZE}, got {len(data)}")
        
        x, y, z = struct.unpack('<3f', data[:cls.SIZE])
        return cls(x=x, y=y, z=z)


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
        
        return cls(
            ping_id=ping_id,
            time=time,
            time_range_zero=time_range_zero,
            parameters=parameters,
            system_info=system_info,
            ascii_sentence_offset=offsets_counts[0],
            ascii_sentence_count=offsets_counts[1],
            port_sidescan_offset=offsets_counts[2],
            port_sidescan_count=offsets_counts[3],
            starboard_sidescan_offset=offsets_counts[4],
            starboard_sidescan_count=offsets_counts[5],
            port_sidescan3d_offset=offsets_counts[6],
            port_sidescan3d_count=offsets_counts[7],
            starboard_sidescan3d_offset=offsets_counts[8],
            starboard_sidescan3d_count=offsets_counts[9],
            port_bathymetry_offset=offsets_counts[10],
            port_bathymetry_count=offsets_counts[11],
            starboard_bathymetry_offset=offsets_counts[12],
            starboard_bathymetry_count=offsets_counts[13],
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
    
    def get_port_sidescan(self) -> np.ndarray:
        """Extract port sidescan samples as uint16 array."""
        if not self.port_sidescan_offset or not self.port_sidescan_count:
            return np.array([], dtype=np.uint16)
        
        offset = self.port_sidescan_offset
        # Sidescan points are stored as SidescanPoint structs (8 bytes each: range + amplitude)
        samples = []
        for _ in range(self.port_sidescan_count):
            range_m, amplitude = struct.unpack('<ff', self._raw_data[offset:offset+8])
            samples.append(amplitude)
            offset += 8
        
        return np.array(samples, dtype=np.float32)
    
    def get_starboard_sidescan(self) -> np.ndarray:
        """Extract starboard sidescan samples as uint16 array."""
        if not self.starboard_sidescan_offset or not self.starboard_sidescan_count:
            return np.array([], dtype=np.uint16)
        
        offset = self.starboard_sidescan_offset
        samples = []
        for _ in range(self.starboard_sidescan_count):
            range_m, amplitude = struct.unpack('<ff', self._raw_data[offset:offset+8])
            samples.append(amplitude)
            offset += 8
        
        return np.array(samples, dtype=np.float32)
    
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
        port_data = np.array([(p.range_m, p.angle_rad) for p in port_points], dtype=np.float32)
        stbd_data = np.array([(p.range_m, p.angle_rad) for p in starboard_points], dtype=np.float32)
        
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
        
        return xyz
    
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
