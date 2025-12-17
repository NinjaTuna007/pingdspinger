#!/usr/bin/env python3
"""
Replay TCP data from pcap file to test the ROS driver.

This script extracts TCP payload from a pcap file and sends it to a local
TCP server socket, allowing you to test the driver without live sonar.
"""

import socket
import subprocess
import sys
import time
import argparse


def stream_tcp_packets(pcap_file, src_port):
    """
    Stream TCP packets from pcap file using tshark (generator).
    
    Yields packets one at a time without loading entire file into memory.
    
    Args:
        pcap_file: Path to pcap file
        src_port: Source port to filter (e.g., 23848 for sonar)
    
    Yields:
        Tuple of (payload_bytes, timestamp)
    """
    cmd = [
        'tshark', '-r', pcap_file,
        '-Y', f'tcp.srcport == {src_port} and tcp.len > 0',
        '-T', 'fields',
        '-e', 'frame.time_relative',
        '-e', 'data'
    ]
    
    try:
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        packet_count = 0
        for line in process.stdout:
            line = line.strip()
            if not line:
                continue
            
            parts = line.split('\t')
            if len(parts) != 2:
                continue
            
            timestamp = float(parts[0])
            hex_data = parts[1]
            payload = bytes.fromhex(hex_data.replace(':', ''))
            
            packet_count += 1
            yield (payload, timestamp)
        
        process.wait()
        
        if process.returncode != 0:
            stderr = process.stderr.read()
            raise RuntimeError(f"tshark failed: {stderr}")
    
    except FileNotFoundError:
        print("Error: tshark not found. Install with: sudo apt install tshark")
        sys.exit(1)


def replay_server(packet_stream, listen_port, speed_multiplier=1.0):
    """
    Create TCP server and replay packets to connected client with original timing.
    
    Args:
        packet_stream: Generator yielding (payload, timestamp) tuples
        listen_port: Port to listen on
        speed_multiplier: Speed multiplier (1.0 = realtime, 2.0 = 2x speed, etc)
    """
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server.bind(('127.0.0.1', listen_port))
        server.listen(1)
        
        print(f"\n✓ Server listening on 127.0.0.1:{listen_port}")
        print("Waiting for ROS driver to connect...")
        print("(Run: ros2 launch pingdspinger 3dss.launch)")
        
        client, addr = server.accept()
        print(f"✓ Client connected from {addr}")
        
        # Give client time to initialize
        time.sleep(0.5)
        
        print(f"\nStreaming packets at {speed_multiplier}x speed...")
        
        sent_bytes = 0
        packet_count = 0
        start_time = time.time()
        prev_timestamp = 0.0
        
        for payload, timestamp in packet_stream:
            try:
                # Wait for next packet time
                if packet_count > 0:
                    delay = (timestamp - prev_timestamp) / speed_multiplier
                    if delay > 0:
                        time.sleep(delay)
                
                client.sendall(payload)
                sent_bytes += len(payload)
                packet_count += 1
                prev_timestamp = timestamp
                
                # Update progress every 100 packets
                if packet_count % 100 == 0:
                    elapsed = time.time() - start_time
                    rate = sent_bytes / elapsed / 1024 if elapsed > 0 else 0
                    print(f"  Sent {packet_count} packets, {sent_bytes/1024:.1f} KB ({rate:.1f} KB/s) - {elapsed:.1f}s", end='\r')
            
            except (BrokenPipeError, ConnectionResetError) as e:
                print(f"\n✗ Client disconnected after {sent_bytes} bytes: {e}")
                break
        
        elapsed = time.time() - start_time
        print(f"\n✓ Replay complete: {packet_count} packets, {sent_bytes/1024:.1f} KB in {elapsed:.1f}s")
        
        # Keep connection open for client to finish processing
        time.sleep(2)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        if 'client' in locals():
            client.close()
        server.close()
        print("Server closed")


def main():
    parser = argparse.ArgumentParser(
        description='Replay sonar TCP data from pcap file',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Basic replay
  %(prog)s network_dump/pingDSP_traffic.pcap
  
  # Custom port and speed
  %(prog)s network_dump/pingDSP_traffic.pcap --port 14001 --speed 10
  
  # Then in another terminal:
  ros2 launch pingdspinger 3dss.launch
  (Update sonar_host to '127.0.0.1' in params.yaml)
        '''
    )
    
    parser.add_argument('pcap_file', help='Path to pcap file')
    parser.add_argument('--port', type=int, default=23848,
                        help='Port to listen on (default: 23848)')
    parser.add_argument('--src-port', type=int, default=23848,
                        help='Source port to filter in pcap (default: 23848)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Replay speed multiplier (1.0=realtime, 2.0=2x, 0.5=half speed)')
    
    args = parser.parse_args()
    
    print(f"Streaming TCP packets from {args.pcap_file}...")
    print("(Starting immediately, no pre-loading)")
    
    # Stream packets directly without loading entire file
    packet_stream = stream_tcp_packets(args.pcap_file, args.src_port)
    
    # Replay with timing
    replay_server(packet_stream, args.port, args.speed)


if __name__ == '__main__':
    main()
