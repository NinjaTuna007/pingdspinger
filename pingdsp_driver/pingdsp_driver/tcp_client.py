#!/usr/bin/env python3
"""
TCP Client for 3DSS-DX Sonar

Handles TCP socket connection to 3DSS-DX sonar and receives streaming data.
"""

import socket
import struct
import logging
from typing import Optional, Tuple
from .dx_structures import DxHeader, DxData


class TcpClient:
    """
    TCP client for connecting to 3DSS-DX sonar.
    
    Handles socket connection, reconnection, and data reception following
    the two-step read protocol (header then data).
    """
    
    def __init__(self, host: str, port: int, timeout: float = 5.0):
        """
        Initialize TCP client.
        
        Args:
            host: IP address or hostname of 3DSS-DX sonar
            port: TCP port (default 14001)
            timeout: Socket timeout in seconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None
        self.connected = False
        
        self.logger = logging.getLogger('TcpClient')
    
    def connect(self) -> bool:
        """
        Connect to the 3DSS-DX sonar.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.host, self.port))
            self.connected = True
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
            except:
                pass
            self.socket = None
            self.connected = False
            self.logger.info("Disconnected")
    
    def recv_exact(self, size: int) -> Optional[bytes]:
        """
        Receive exact number of bytes from socket.
        
        Args:
            size: Number of bytes to receive
        
        Returns:
            Bytes received, or None if error/timeout
        """
        if not self.connected or not self.socket:
            return None
        
        data = bytearray()
        remaining = size
        
        try:
            while remaining > 0:
                chunk = self.socket.recv(remaining)
                if not chunk:
                    # Connection closed by remote
                    self.logger.warning("Connection closed by remote")
                    self.connected = False
                    return None
                
                data.extend(chunk)
                remaining -= len(chunk)
            
            return bytes(data)
        
        except socket.timeout:
            self.logger.debug("Socket timeout during recv")
            return None
        
        except socket.error as e:
            self.logger.error(f"Socket error during recv: {e}")
            self.connected = False
            return None
    
    def resync_to_preamble(self) -> bool:
        """
        Search the TCP stream for the next valid preamble.
        
        This is called when we lose synchronization to realign the stream.
        Reads byte-by-byte until we find a valid 16-byte preamble.
        
        Returns:
            True if preamble found and stream is now aligned, False otherwise
        """
        PREAMBLE = bytes([0x50, 0x49, 0x4e, 0x47, 0x27, 0x2b, 0x3a, 0xd8,
                         0x74, 0x2a, 0x1c, 0x33, 0xe9, 0xb0, 0x73, 0xb1])
        
        buffer = bytearray()
        max_search = 1024 * 1024  # Search up to 1MB before giving up
        
        self.logger.info("Lost sync - searching for next preamble...")
        
        for _ in range(max_search):
            try:
                byte = self.socket.recv(1)
                if not byte:
                    self.logger.warning("Connection closed during resync")
                    self.connected = False
                    return False
                
                buffer.append(byte[0])
                
                # Keep only last 16 bytes in buffer
                if len(buffer) > 16:
                    buffer.pop(0)
                
                # Check if we have a valid preamble
                if len(buffer) == 16 and bytes(buffer) == PREAMBLE:
                    self.logger.info("Resync successful - found preamble!")
                    return True
                    
            except socket.timeout:
                continue
            except socket.error as e:
                self.logger.error(f"Socket error during resync: {e}")
                self.connected = False
                return False
        
        self.logger.error("Resync failed - no preamble found in 1MB")
        return False
    
    def read_ping(self) -> Optional[Tuple[DxHeader, DxData]]:
        """
        Read one complete ping (header + data) from the TCP stream.
        
        This implements the two-step read protocol:
        1. Read DxHeader (20 bytes) to get data length
        2. Read DxData (variable length)
        
        Returns:
            (DxHeader, DxData) tuple if successful, None otherwise
        """
        if not self.connected:
            return None
        
        # Step 1: Read header (20 bytes)
        header_bytes = self.recv_exact(DxHeader.SIZE)
        if not header_bytes:
            return None
        
        try:
            header = DxHeader.from_bytes(header_bytes)
        except Exception as e:
            self.logger.error(f"Failed to parse header: {e}")
            # Try to resync
            if self.resync_to_preamble():
                # Read remaining 4 bytes of header (data_count)
                remaining = self.recv_exact(4)
                if remaining:
                    data_count = struct.unpack('<I', remaining)[0]
                    # Reconstruct header with valid preamble
                    header = DxHeader(preamble=bytes([0x50, 0x49, 0x4e, 0x47, 0x27, 0x2b, 0x3a, 0xd8,
                                                      0x74, 0x2a, 0x1c, 0x33, 0xe9, 0xb0, 0x73, 0xb1]),
                                     data_count=data_count)
                else:
                    return None
            else:
                return None
        
        # Validate preamble
        if not header.is_valid():
            # Show first 8 bytes of invalid preamble for debugging
            preamble_hex = header.preamble[:8].hex()
            self.logger.warning(f"Invalid preamble: {preamble_hex}... (expected PING...)")
            
            # Try to resync
            if self.resync_to_preamble():
                # Read remaining 4 bytes of header (data_count)
                remaining = self.recv_exact(4)
                if remaining:
                    data_count = struct.unpack('<I', remaining)[0]
                    # Reconstruct header with valid preamble
                    header = DxHeader(preamble=bytes([0x50, 0x49, 0x4e, 0x47, 0x27, 0x2b, 0x3a, 0xd8,
                                                      0x74, 0x2a, 0x1c, 0x33, 0xe9, 0xb0, 0x73, 0xb1]),
                                     data_count=data_count)
                else:
                    return None
            else:
                return None
        
        # Step 2: Read data (variable length)
        if header.data_count == 0:
            self.logger.warning("Header indicates zero data length")
            return None
        
        self.logger.debug(f"Reading {header.data_count} bytes of data...")
        data_bytes = self.recv_exact(header.data_count)
        if not data_bytes:
            self.logger.error("Failed to receive data bytes")
            return None
        
        if len(data_bytes) != header.data_count:
            self.logger.error(f"Data size mismatch: expected {header.data_count}, got {len(data_bytes)}")
            return None
        
        try:
            data = DxData.from_bytes(data_bytes)
            self.logger.debug(f"Successfully parsed ping {data.ping_id}")
        except Exception as e:
            self.logger.error(f"Failed to parse data: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return None
        
        return (header, data)
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
