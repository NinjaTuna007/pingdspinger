#!/usr/bin/env python3
"""Replay SBG Ellipse UDP traffic from a pcap to a local sbg_driver.

The PingDSP rig's SBG Ellipse streams sbgECom binary over UDP (default port
24333, see ``config/sbg_device_udp.yaml``). This script pulls the UDP payloads
out of a capture with ``tshark`` and re-emits them as UDP datagrams to a target
host/port, preserving the original inter-packet timing. Point ``sbg_device`` at
``0.0.0.0:<port>`` and it will ingest the replay exactly as if it were live.

Example::

    # Terminal 1: bring up the SBG stack (sbg_device listens on 0.0.0.0:24333)
    ros2 launch pingdsp_sbg sbg.launch

    # Terminal 2: replay a capture to the default 127.0.0.1:24333
    SCR="$(ros2 pkg prefix pingdsp_sbg)/share/pingdsp_sbg/scripts"
    python3 "$SCR/replay_sbg_udp.py" network_dump/asko_survey.pcap

Or end-to-end (replayer + full stack) via the test launch::

    ros2 launch pingdsp_sbg test_sbg.launch pcap_file:=/abs/path/to/capture.pcap
"""

import argparse
import socket
import subprocess
import sys
import time


def stream_udp_payloads(pcap_file, port):
    """Yield (payload_bytes, frame_time_relative) for UDP packets on ``port``.

    Streams via ``tshark`` so large captures are not loaded into memory.
    """
    cmd = [
        'tshark', '-r', pcap_file,
        '-Y', f'udp.port == {port} and udp.length > 8',
        '-T', 'fields',
        '-e', 'frame.time_relative',
        '-e', 'udp.payload',
    ]
    try:
        process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    except FileNotFoundError:
        print('Error: tshark not found. Install with: sudo apt install tshark')
        sys.exit(1)

    for line in process.stdout:
        line = line.strip()
        if not line:
            continue
        parts = line.split('\t')
        if len(parts) != 2 or not parts[1]:
            continue
        try:
            timestamp = float(parts[0])
            payload = bytes.fromhex(parts[1].replace(':', ''))
        except ValueError:
            continue
        yield (payload, timestamp)

    process.wait()
    if process.returncode not in (0, None):
        stderr = process.stderr.read()
        raise RuntimeError(f'tshark failed: {stderr}')


def replay(payload_stream, host, port, speed_multiplier=1.0):
    """Send each UDP payload to host:port, honouring original packet timing."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (host, port)
    print(f'Replaying SBG UDP -> {host}:{port} at {speed_multiplier}x')

    sent_bytes = 0
    packet_count = 0
    start_time = time.time()
    prev_timestamp = 0.0
    try:
        for payload, timestamp in payload_stream:
            if packet_count > 0:
                delay = (timestamp - prev_timestamp) / speed_multiplier
                if delay > 0:
                    time.sleep(delay)
            sock.sendto(payload, target)
            sent_bytes += len(payload)
            packet_count += 1
            prev_timestamp = timestamp
            if packet_count % 200 == 0:
                elapsed = time.time() - start_time
                print(f'  sent {packet_count} datagrams, '
                      f'{sent_bytes / 1024:.1f} KB ({elapsed:.1f}s)', end='\r')
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        sock.close()
        elapsed = time.time() - start_time
        print(f'\nReplay complete: {packet_count} datagrams, '
              f'{sent_bytes / 1024:.1f} KB in {elapsed:.1f}s')


def main():
    parser = argparse.ArgumentParser(
        description='Replay SBG Ellipse UDP data from a pcap file')
    parser.add_argument('pcap_file', help='Path to pcap/pcapng file')
    parser.add_argument('--host', default='127.0.0.1',
                        help='Destination host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=24333,
                        help='UDP port to filter and send to (default: 24333)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Speed multiplier (1.0=realtime, 10=10x)')
    args = parser.parse_args()

    stream = stream_udp_payloads(args.pcap_file, args.port)
    replay(stream, args.host, args.port, args.speed)


if __name__ == '__main__':
    main()
