#!/usr/bin/env python3
"""
ROS 2 Control Node for PingDSP 3DSS-DX Sonar

Provides ROS 2 services to control sonar settings via TCP control interface.
Based on 3DSS-DX Control Command Interface Guide 1.4.

Control connection is separate from data streaming connection.
"""

import rclpy
from rclpy.node import Node
import socket
import re
import logging
from typing import Optional

from pingdsp_msg.srv import (
    AppControl,
    SidescanSettings,
    Sidescan3DSettings,
    BathymetrySettings,
    TransmitSettings,
    AcquisitionSettings,
    CommitSettings,
    SonarControl,
    FileControl,
    RecordControl,
    BaudSettings,
    SoundVelocity
)


class SonarControlNode(Node):
    """ROS 2 node for controlling 3DSS-DX sonar settings via TCP control interface."""
    
    def __init__(self):
        super().__init__('sonar_control_node')
        
        # Declare parameters
        self.declare_parameter('sonar_host', '192.168.1.100')
        self.declare_parameter('control_port', 23840)  # Control port for PingDSP 3DSS-DX
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('reconnect_on_disconnect', True)
        
        # Get parameters
        self.sonar_host = self.get_parameter('sonar_host').value
        self.control_port = self.get_parameter('control_port').value
        self.timeout = self.get_parameter('timeout').value
        self.reconnect_on_disconnect = self.get_parameter('reconnect_on_disconnect').value
        
        # Setup logging
        self.logger = self.get_logger()
        
        # Control socket
        self.control_socket: Optional[socket.socket] = None
        self.connected = False
        
        # Create services based on 3DSS-DX Control Command Interface Guide
        self.srv_app_control = self.create_service(
            AppControl,
            'sonar/app_control',
            self.app_control_callback
        )
        
        self.srv_sidescan = self.create_service(
            SidescanSettings,
            'sonar/sidescan',
            self.sidescan_callback
        )
        
        self.srv_sidescan3d = self.create_service(
            Sidescan3DSettings,
            'sonar/sidescan3d',
            self.sidescan3d_callback
        )
        
        self.srv_bathymetry = self.create_service(
            BathymetrySettings,
            'sonar/bathymetry',
            self.bathymetry_callback
        )
        
        self.srv_transmit = self.create_service(
            TransmitSettings,
            'sonar/transmit',
            self.transmit_callback
        )
        
        self.srv_acquisition = self.create_service(
            AcquisitionSettings,
            'sonar/acquisition',
            self.acquisition_callback
        )
        
        self.srv_commit = self.create_service(
            CommitSettings,
            'sonar/commit',
            self.commit_callback
        )
        
        self.srv_sonar_control = self.create_service(
            SonarControl,
            'sonar/control',
            self.sonar_control_callback
        )
        
        self.srv_file_control = self.create_service(
            FileControl,
            'sonar/file',
            self.file_control_callback
        )
        
        self.srv_record_control = self.create_service(
            RecordControl,
            'sonar/record',
            self.record_callback
        )
        
        self.srv_baud = self.create_service(
            BaudSettings,
            'sonar/baud',
            self.baud_callback
        )
        
        self.srv_sound_velocity = self.create_service(
            SoundVelocity,
            'sonar/sound_velocity',
            self.sound_velocity_callback
        )
        
        # Connect to sonar control interface
        self.connect_to_sonar()
        
        self.logger.info(f"Sonar Control Node initialized")
        self.logger.info(f"  Control endpoint: {self.sonar_host}:{self.control_port}")
        self.logger.info(f"Services available:")
        self.logger.info(f"  - /sonar/app_control")
        self.logger.info(f"  - /sonar/sidescan")
        self.logger.info(f"  - /sonar/sidescan3d")
        self.logger.info(f"  - /sonar/bathymetry")
        self.logger.info(f"  - /sonar/transmit")
        self.logger.info(f"  - /sonar/acquisition")
        self.logger.info(f"  - /sonar/commit")
        self.logger.info(f"  - /sonar/control")
        self.logger.info(f"  - /sonar/file")
        self.logger.info(f"  - /sonar/record")
        self.logger.info(f"  - /sonar/baud")
        self.logger.info(f"  - /sonar/sound_velocity")
    
    def connect_to_sonar(self) -> bool:
        """Connect to sonar control interface."""
        try:
            self.logger.info(f"Connecting to control interface at {self.sonar_host}:{self.control_port}")
            self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.control_socket.settimeout(self.timeout)
            self.control_socket.connect((self.sonar_host, self.control_port))
            self.connected = True
            self.logger.info("Connected to sonar control interface")
            return True
        
        except socket.timeout:
            self.logger.error(f"Connection timeout to {self.sonar_host}:{self.control_port}")
            self.connected = False
            return False
        
        except socket.error as e:
            self.logger.error(f"Connection error: {e}")
            try:
                self.control_socket.close()
            except:
                pass
            self.control_socket = None
            self.connected = False
            self.logger.info("Disconnected from control interface")
    
    def send_command(self, command: str) -> Optional[str]:
        """
        Send ASCII command to sonar and receive response.
        
        Args:
            command: ASCII command string (e.g., "SET RANGE 50")
        
        Returns:
            Response string or None if error
        """
        if not self.connected:
            if self.reconnect_on_disconnect:
                self.logger.warning("Not connected, attempting reconnection...")
                if not self.connect_to_sonar():
                    return None
            else:
                return None
        
        try:
            # Send command (add newline/carriage return as per protocol)
            cmd_bytes = (command + "\r\n").encode('utf-8')
            self.control_socket.sendall(cmd_bytes)
            self.logger.debug(f"Sent: {command}")
            
            # Receive response
            response_bytes = self.control_socket.recv(4096)
            
            if not response_bytes:
                self.logger.warning("Received empty response")
                return ""
            
            # Log raw bytes for debugging
            self.logger.debug(f"Raw bytes: {response_bytes.hex()}")
            
            # Decode as UTF-8 (sonar sends UTF-8 BOM 0xEFBBBF)
            try:
                response = response_bytes.decode('utf-8').strip()
                
                # Strip UTF-8 BOM if present
                if response.startswith('\ufeff'):
                    response = response[1:]
                    self.logger.debug("Stripped UTF-8 BOM from response")
                
            except UnicodeDecodeError as e:
                # Fall back to latin-1 which never fails
                response = response_bytes.decode('latin-1', errors='replace').strip()
                self.logger.warning(f"UTF-8 decode failed: {e}")
                self.logger.warning(f"Received bytes: {response_bytes.hex()}")
                self.logger.warning(f"Decoded as latin-1: {repr(response)}")
            
            self.logger.debug(f"Received: {repr(response)}")
            
            return response
        
        except socket.timeout:
            self.logger.error("Command timeout")
            self.connected = False
            return None
        
        except socket.error as e:
            self.logger.error(f"Socket error: {e}")
            self.connected = False
            return None
        
        except Exception as e:
            self.logger.error(f"Unexpected error: {e}")
            return None
    
    def parse_response(self, response: str, expected_pattern: str = None) -> tuple[bool, str]:
        """
        Parse command response to determine success/failure.
        
        Args:
            response: Response string from sonar
            expected_pattern: Optional regex pattern to match for success
        
        Returns:
            Tuple of (success, message)
        """
        if response is None:
            return False, "No response from sonar (connection issue)"
        
        # Check for common error indicators
        if "ERROR" in response.upper() or "FAIL" in response.upper():
            return False, response
        
        # Check for common success indicators
        if "OK" in response.upper() or "SUCCESS" in response.upper():
            return True, response
        
        # If expected pattern provided, check it
        if expected_pattern and re.search(expected_pattern, response, re.IGNORECASE):
            return True, response
        
        # Default: assume success if we got a response
        return True, response
    
    # ========================================================================
    # Service callbacks based on 3DSS-DX Control Command Interface Guide
    # ========================================================================
    
    def app_control_callback(self, request, response):
        """
        Handle app control service request.
        Section 3.3: APP command
        """
        command = request.command.lower()
        
        if command == "init":
            if request.mode:
                cmd = f"app --init --mode={request.mode}"
            else:
                cmd = "app --init"
        elif command == "mode":
            if not request.mode:
                response.success = False
                response.message = "Mode must be specified for mode command"
                return response
            cmd = f"app --init --mode={request.mode}"
        elif command == "exit":
            cmd = "app --exit"
        elif command == "status" or command == "get":
            cmd = "app"
        else:
            response.success = False
            response.message = f"Unknown command: {command}. Use 'init', 'mode', 'exit', or 'status'"
            return response
        
        self.logger.info(f"App control: {cmd}")
        result = self.send_command(cmd)
        response.success, response.message = self.parse_response(result)
        
        # Parse current mode from response if querying status
        if command in ["status", "get"] and result:
            # Response format: "okay (mode=fileprocess)"
            import re
            match = re.search(r'mode=(\w+)', result)
            if match:
                response.current_mode = match.group(1)
        
        return response
    
    def sidescan_callback(self, request, response):
        """
        Handle sidescan settings service request.
        Section 3.4: SIDESCAN command
        """
        command = request.command.lower()
        
        if command == "get":
            # Query current settings
            if request.side.lower() == "port":
                cmd = "sidescan --port"
            elif request.side.lower() == "starboard" or request.side.lower() == "stbd":
                cmd = "sidescan --stbd"
            else:
                cmd = "sidescan"
        elif command == "set":
            # Build set command
            cmd_parts = ["sidescan"]
            
            if request.mode:
                cmd_parts.append(f"--mode={request.mode}")
            if request.method:
                cmd_parts.append(f"--method={request.method}")
            if request.beams:
                cmd_parts.append(f"--beams={request.beams}")
            
            # Add side specification
            side = request.side.lower()
            if side == "port":
                cmd_parts.append("--port")
            elif side == "starboard" or side == "stbd":
                cmd_parts.append("--stbd")
            # Both sides if neither specified
            
            cmd = " ".join(cmd_parts)
        else:
            response.success = False
            response.message = f"Unknown command: {command}. Use 'get' or 'set'"
            return response
        
        self.logger.info(f"Sidescan: {cmd}")
        result = self.send_command(cmd)
        response.success, response.message = self.parse_response(result)
        
        return response
    
    def sidescan3d_callback(self, request, response):
        """
        Handle sidescan 3D settings service request.
        Section 3.5: SIDESCAN3D command
        """
        command = request.command.lower()
        
        if command == "get":
            # Query current settings
            if request.side.lower() == "port":
                cmd = "sidescan3d --port"
            elif request.side.lower() == "starboard" or request.side.lower() == "stbd":
                cmd = "sidescan3d --stbd"
            else:
                cmd = "sidescan3d"
        elif command == "set":
            # Build set command
            cmd_parts = ["sidescan3d"]
            
            if request.angles > 0:
                cmd_parts.append(f"--angles={request.angles}")
            if request.smoothing >= 0:
                cmd_parts.append(f"--smoothing={request.smoothing}")
            if request.threshold != 0.0:
                cmd_parts.append(f"--threshold={request.threshold}")
            if request.tolerance != 0.0:
                cmd_parts.append(f"--tolerance={request.tolerance}")
            if request.mindepth != 0.0:
                cmd_parts.append(f"--mindepth={request.mindepth}")
            if request.maxdepth != 0.0:
                cmd_parts.append(f"--maxdepth={request.maxdepth}")
            if request.swath != 0.0:
                cmd_parts.append(f"--swath={request.swath}")
            if request.amp != 0.0:
                cmd_parts.append(f"--amp={request.amp}")
            
            # Add side specification (image filtering applies to both sides)
            side = request.side.lower()
            if side == "port":
                cmd_parts.append("--port")
            elif side == "starboard" or side == "stbd":
                cmd_parts.append("--stbd")
            
            cmd = " ".join(cmd_parts)
        else:
            response.success = False
            response.message = f"Unknown command: {command}. Use 'get' or 'set'"
            return response
        
        self.logger.info(f"Sidescan3D: {cmd}")
        result = self.send_command(cmd)
        response.success, response.message = self.parse_response(result)
        
        return response
    
    def bathymetry_callback(self, request, response):
        """
        Handle bathymetry settings service request.
        Section 3.6: BATHYMETRY command
        """
        command = request.command.lower()
        
        if command == "get":
            cmd = "bathymetry"
        elif command == "set":
            # Build set command
            cmd_parts = ["bathymetry"]
            
            if request.mindepth != 0.0:
                cmd_parts.append(f"--mindepth={request.mindepth}")
            if request.maxdepth != 0.0:
                cmd_parts.append(f"--maxdepth={request.maxdepth}")
            if request.swath != 0.0:
                cmd_parts.append(f"--swath={request.swath}")
            
            # Binning parameters
            if request.binning_mode:
                binning = f"--binning={request.binning_mode}:{request.binning_count}:{request.binning_width}"
                cmd_parts.append(binning)
            
            # Bottom track parameters
            if request.bottomtrack_mode:
                bottomtrack = (f"--bottomtrack={request.bottomtrack_mode}:"
                             f"{request.bottomtrack_cells}:{request.bottomtrack_width}:"
                             f"{request.bottomtrack_height}:{request.bottomtrack_heightp}:"
                             f"{request.bottomtrack_alpha}")
                cmd_parts.append(bottomtrack)
            
            cmd = " ".join(cmd_parts)
        else:
            response.success = False
            response.message = f"Unknown command: {command}. Use 'get' or 'set'"
            return response
        
        self.logger.info(f"Bathymetry: {cmd}")
        result = self.send_command(cmd)
        response.success, response.message = self.parse_response(result)
        
        return response
    
    def transmit_callback(self, request, response):
        """
        Handle transmit settings service request.
        Section 3.7: TRANSMIT command
        """
        command = request.command.lower()
        
        if command == "get":
            # Query current settings
            if request.side.lower() == "port":
                cmd = "transmit --port"
            elif request.side.lower() == "starboard" or request.side.lower() == "stbd":
                cmd = "transmit --stbd"
            else:
                cmd = "transmit"
        elif command == "set":
            # Build set command
            cmd_parts = ["transmit"]
            
            if request.pulse:
                cmd_parts.append(f"--pulse={request.pulse}")
            if request.power > 0:
                cmd_parts.append(f"--power={request.power}")
            if request.beamwidth > 0:
                cmd_parts.append(f"--beamwidth={request.beamwidth}")
            if request.angle != 999:  # Use 999 as sentinel for "not set"
                cmd_parts.append(f"--angle={request.angle}")
            
            # Add side specification
            side = request.side.lower()
            if side == "port":
                cmd_parts.append("--port")
            elif side == "starboard" or side == "stbd":
                cmd_parts.append("--stbd")
            
            cmd = " ".join(cmd_parts)
        else:
            response.success = False
            response.message = f"Unknown command: {command}. Use 'get' or 'set'"
            return response
        
        self.logger.info(f"Transmit: {cmd}")
        result = self.send_command(cmd)
        response.success, response.message = self.parse_response(result)
        
        return response
    
    def acquisition_callback(self, request, response):
        """
        Handle acquisition settings service request.
        Section 3.8: ACQUISITION command
        """
        command = request.command.lower()
        
        if command == "get":
            cmd = "acquisition"
        elif command == "set":
            # Build set command
            cmd_parts = ["acquisition"]
            
            if request.range > 0:
                cmd_parts.append(f"--range={request.range}")
            if request.dutycycle > 0:
                cmd_parts.append(f"--dutycycle={request.dutycycle}")
            if request.trigger:
                cmd_parts.append(f"--trigger={request.trigger}")
            if request.maxdepth != 0.0:
                cmd_parts.append(f"--maxdepth={request.maxdepth}")
            if request.env:
                cmd_parts.append(f"--env={request.env}")
            if request.priority:
                cmd_parts.append(f"--priority={request.priority}")
            
            cmd = " ".join(cmd_parts)
        else:
            response.success = False
            response.message = f"Unknown command: {command}. Use 'get' or 'set'"
            return response
        
        self.logger.info(f"Acquisition: {cmd}")
        result = self.send_command(cmd)
        response.success, response.message = self.parse_response(result)
        
        return response
    
    def commit_callback(self, request, response):
        """
        Handle commit service request.
        Section 3.9: COMMIT command
        """
        self.logger.info("Committing changes to sonar/file processor")
        cmd = "commit"
        result = self.send_command(cmd)
        response.success, response.message = self.parse_response(result)
        
        return response
    
    def sonar_control_callback(self, request, response):
        """
        Handle sonar control service request.
        Section 3.10: SONAR command
        """
        command = request.command.lower()
        
        valid_commands = ["connect", "disconnect", "updatetime", "run", "stop", "status"]
        if command not in valid_commands:
            response.success = False
            response.message = f"Unknown command: {command}. Use: {', '.join(valid_commands)}"
            return response
        
        cmd = f"sonar --{command}"
        self.logger.info(f"Sonar control: {cmd}")
        result = self.send_command(cmd)
        response.success, response.message = self.parse_response(result)
        
        # Parse status response if querying status
        if command == "status" and result:
            # Response format: "okay (id=A02-12345678 pings=9401 ratehz=5.1 ns=0/0/9401)"
            import re
            id_match = re.search(r'id=([\w-]+)', result)
            pings_match = re.search(r'pings=(\d+)', result)
            rate_match = re.search(r'ratehz=([\d.]+)', result)
            ns_match = re.search(r'ns=([\d/]+)', result)
            
            if id_match:
                response.sonar_id = id_match.group(1)
            if pings_match:
                response.pings = int(pings_match.group(1))
            if rate_match:
                response.ratehz = float(rate_match.group(1))
            if ns_match:
                response.network_status = ns_match.group(1)
        
        return response
    
    def file_control_callback(self, request, response):
        """
        Handle file control service request.
        Section 3.11: FILE command
        """
        command = request.command.lower()
        
        if command == "open":
            if not request.filename:
                response.success = False
                response.message = "Filename must be specified for open command"
                return response
            # Quote filename if it contains spaces
            filename = request.filename
            if ' ' in filename:
                filename = f'"{filename}"'
            cmd = f"file --open --filename={filename}"
        elif command == "close":
            cmd = "file --close"
        elif command == "play":
            cmd = "file --play"
        elif command == "stop":
            cmd = "file --stop"
        elif command == "status":
            cmd = "file --status"
        elif command == "speed":
            if request.speed <= 0:
                response.success = False
                response.message = "Speed must be specified and positive"
                return response
            cmd = f"file --speed={request.speed}"
        else:
            response.success = False
            response.message = f"Unknown command: {command}. Use: open, close, play, stop, status, speed"
            return response
        
        self.logger.info(f"File control: {cmd}")
        result = self.send_command(cmd)
        response.success, response.message = self.parse_response(result)
        
        # Parse status response
        if command == "status" and result:
            # Response format: "okay (file="lake union" size=491892831 ratehz=9.4 ratert=2.0)"
            import re
            file_match = re.search(r'file="([^"]+)"', result)
            size_match = re.search(r'size=(\d+)', result)
            ratehz_match = re.search(r'ratehz=([\d.]+)', result)
            ratert_match = re.search(r'ratert=([\d.]+)', result)
            
            if file_match:
                response.file_name = file_match.group(1)
            if size_match:
                response.file_size = int(size_match.group(1))
            if ratehz_match:
                response.ratehz = float(ratehz_match.group(1))
            if ratert_match:
                response.ratert = float(ratert_match.group(1))
        
        return response
    
    def record_callback(self, request, response):
        """
        Handle record control service request.
        Section 3.12: RECORD command
        """
        command = request.command.lower()
        
        if command == "start":
            if not request.filename:
                response.success = False
                response.message = "Filename must be specified for start command"
                return response
            # Quote filename if it contains spaces
            filename = request.filename
            if ' ' in filename:
                filename = f'"{filename}"'
            
            cmd_parts = ["record", "--start"]
            if request.overwrite:
                cmd_parts.append("--overwrite")
            cmd_parts.append(f"--filename={filename}")
            if request.mode > 0:
                cmd_parts.append(f"--mode={request.mode}")
            
            cmd = " ".join(cmd_parts)
        elif command == "stop":
            cmd = "record --stop"
        elif command == "status":
            cmd = "record --status"
        else:
            response.success = False
            response.message = f"Unknown command: {command}. Use: start, stop, status"
            return response
        
        self.logger.info(f"Record control: {cmd}")
        result = self.send_command(cmd)
        response.success, response.message = self.parse_response(result)
        
        # Parse status response
        if command == "status" and result:
            # Response format: "okay (file="c:\data\lake union2.3dss-dx" size=190218392 pings=381)"
            import re
            file_match = re.search(r'file="([^"]+)"', result)
            size_match = re.search(r'size=(\d+)', result)
            pings_match = re.search(r'pings=(\d+)', result)
            
            if file_match:
                response.file_name = file_match.group(1)
            if size_match:
                response.file_size = int(size_match.group(1))
            if pings_match:
                response.pings = int(pings_match.group(1))
        
        return response
    
    def baud_callback(self, request, response):
        """
        Handle baud settings service request.
        Section 3.13: BAUD command
        """
        command = request.command.lower()
        
        if command == "get":
            cmd = "baud"
        elif command == "set":
            device = request.device.lower()
            if device not in ["gps", "mru"]:
                response.success = False
                response.message = "Device must be 'gps' or 'mru'"
                return response
            
            if request.baudrate <= 0:
                response.success = False
                response.message = "Baudrate must be specified"
                return response
            
            cmd = f"baud --{device}={request.baudrate}"
        else:
            response.success = False
            response.message = f"Unknown command: {command}. Use 'get' or 'set'"
            return response
        
        self.logger.info(f"Baud settings: {cmd}")
        result = self.send_command(cmd)
        response.success, response.message = self.parse_response(result)
        
        # Parse response for current baud rates
        if command == "get" and result:
            # Response format: "okay (gps=57600 mru=38400)"
            import re
            gps_match = re.search(r'gps=(\d+)', result)
            mru_match = re.search(r'mru=(\d+)', result)
            
            if gps_match:
                response.gps_baud = int(gps_match.group(1))
            if mru_match:
                response.mru_baud = int(mru_match.group(1))
        
        return response
    
    def sound_velocity_callback(self, request, response):
        """
        Handle sound velocity service request.
        Section 3.2: sv command
        Command: sv [--bulk=<value> | --face=<value>]
        Response: okay (bulk=1520 face=1505.5)
        """
        command = request.command.lower()
        
        if command == "get":
            cmd = "sv"
        elif command == "set":
            # Build command with bulk and/or face parameters
            options = []
            if request.bulk_velocity > 0:
                if not (1300 <= request.bulk_velocity <= 2500):
                    response.success = False
                    response.message = "Bulk velocity must be between 1300 and 2500 m/s"
                    return response
                options.append(f"--bulk={request.bulk_velocity:.1f}")
            if request.face_velocity > 0:
                if not (1300 <= request.face_velocity <= 2500):
                    response.success = False
                    response.message = "Face velocity must be between 1300 and 2500 m/s"
                    return response
                options.append(f"--face={request.face_velocity:.1f}")
            
            if not options:
                response.success = False
                response.message = "Must specify at least one of bulk_velocity or face_velocity"
                return response
            
            cmd = f"sv {' '.join(options)}"
        else:
            response.success = False
            response.message = f"Unknown command: {command}. Use 'get' or 'set'"
            return response
        
        self.logger.info(f"Sound velocity: {cmd}")
        result = self.send_command(cmd)
        response.success, response.message = self.parse_response(result)
        
        # Parse response for current velocities
        # Response format: okay (bulk=1520 face=1505.5)
        if result:
            import re
            bulk_match = re.search(r'bulk[=\s]+(\d+\.?\d*)', result, re.IGNORECASE)
            face_match = re.search(r'face[=\s]+(\d+\.?\d*)', result, re.IGNORECASE)
            if bulk_match:
                response.current_bulk = float(bulk_match.group(1))
            if face_match:
                response.current_face = float(face_match.group(1))
        
        return response
    
    def destroy_node(self):
        """Clean shutdown."""
        self.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SonarControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
