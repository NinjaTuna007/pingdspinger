#!/usr/bin/env python3
"""Test DxData parsing directly"""

import subprocess
import sys
sys.path.insert(0, '/home/shekharu/colcon_ws/src/my_pkgs/pingdspinger/pingdspinger')

from dx_structures import DxHeader, DxData

# Extract TCP stream
pcap_file = "/home/shekharu/colcon_ws/src/my_pkgs/pingdspinger/network_dump/pingDSP_sample.pcap"

print("Extracting TCP stream...")
result = subprocess.run(
    ['tshark', '-r', pcap_file, '-Y', 'tcp.srcport == 23848 and tcp.len > 0', 
     '-T', 'fields', '-e', 'data'],
    capture_output=True, text=True
)

stream = bytearray()
for hex_line in result.stdout.strip().split('\n'):
    if hex_line:
        stream.extend(bytes.fromhex(hex_line.replace(':', '')))

print(f"Stream size: {len(stream)} bytes")
print()

# Parse first packet
offset = 0

# Read header
print("=== Parsing DxHeader ===")
header_bytes = bytes(stream[offset:offset+20])
print(f"Header bytes (20): {header_bytes[:20].hex()}")

try:
    header = DxHeader.from_bytes(header_bytes)
    print(f"✓ Header parsed successfully")
    print(f"  Preamble valid: {header.is_valid()}")
    print(f"  Data count: {header.data_count}")
except Exception as e:
    print(f"✗ Header parse failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

offset += 20

# Read data
print()
print("=== Parsing DxData ===")
data_bytes = bytes(stream[offset:offset+header.data_count])
print(f"Data size: {len(data_bytes)} bytes (expected {header.data_count})")
print(f"First 100 bytes: {data_bytes[:100].hex()}")
print()

try:
    data = DxData.from_bytes(data_bytes)
    print(f"✓ Data parsed successfully!")
    print(f"  Ping ID: {data.ping_id}")
    print(f"  Time: {data.time.seconds}s + {data.time.nanoseconds}ns")
    print(f"  Range: {data.parameters.range_m}m")
    print(f"  Sonar ID: {data.system_info.sonar_id}")
    print(f"  Port bathymetry points: {data.port_bathymetry_count}")
    print(f"  Starboard bathymetry points: {data.starboard_bathymetry_count}")
except Exception as e:
    print(f"✗ Data parse failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print()
print("=== SUCCESS ===")
