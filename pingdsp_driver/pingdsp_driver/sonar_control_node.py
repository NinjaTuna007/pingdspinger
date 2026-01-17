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
    SetSonarRange,
    SetSonarGain,
    SetSonarPower,
    SetSoundVelocity,
    GetSonarSettings,
    SetTriggerMode
)
from pingdsp_msg.msg import SonarSettings


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
        
        # Create services
        self.srv_set_range = self.create_service(
            SetSonarRange,
            'sonar/set_range',
            self.set_range_callback
        )
        
        self.srv_set_gain = self.create_service(
            SetSonarGain,
            'sonar/set_gain',
            self.set_gain_callback
        )
        
        self.srv_set_power = self.create_service(
            SetSonarPower,
            'sonar/set_power',
            self.set_power_callback
        )
        
        self.srv_set_sound_velocity = self.create_service(
            SetSoundVelocity,
            'sonar/set_sound_velocity',
            self.set_sound_velocity_callback
        )
        
        self.srv_get_settings = self.create_service(
            GetSonarSettings,
            'sonar/get_settings',
            self.get_settings_callback
        )
        
        self.srv_set_trigger = self.create_service(
            SetTriggerMode,
            'sonar/set_trigger_mode',
            self.set_trigger_callback
        )
        
        # Connect to sonar control interface
        self.connect_to_sonar()
        
        self.logger.info(f"Sonar Control Node initialized")
        self.logger.info(f"  Control endpoint: {self.sonar_host}:{self.control_port}")
        self.logger.info(f"Services available:")
        self.logger.info(f"  - /sonar/set_range")
        self.logger.info(f"  - /sonar/set_gain")
        self.logger.info(f"  - /sonar/set_power")
        self.logger.info(f"  - /sonar/set_sound_velocity")
        self.logger.info(f"  - /sonar/get_settings")
        self.logger.info(f"  - /sonar/set_trigger_mode")
    
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
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from sonar control interface."""
        if self.control_socket:
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
            cmd_bytes = (command + "\r\n").encode('ascii')
            self.control_socket.sendall(cmd_bytes)
            self.logger.debug(f"Sent: {command}")
            
            # Receive response
            response = self.control_socket.recv(4096).decode('ascii').strip()
            self.logger.debug(f"Received: {response}")
            
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
    
    # Service callbacks
    
    def set_range_callback(self, request, response):
        """Handle set_range service request."""
        self.logger.info(f"Setting range to {request.range} meters")
        
        # Send command (format from control interface guide)
        command = f"SET RANGE {request.range}"
        result = self.send_command(command)
        
        response.success, response.message = self.parse_response(result)
        return response
    
    def set_gain_callback(self, request, response):
        """Handle set_gain service request."""
        side = request.side.upper()
        if side not in ["PORT", "STARBOARD"]:
            response.success = False
            response.message = "Invalid side. Must be 'port' or 'starboard'"
            return response
        
        self.logger.info(f"Setting {side} gain: constant={request.constant}, "
                        f"linear={request.linear}, logarithmic={request.logarithmic}")
        
        # Send commands for each gain component
        commands = [
            f"SET {side} GAIN CONSTANT {request.constant}",
            f"SET {side} GAIN LINEAR {request.linear}",
            f"SET {side} GAIN LOGARITHMIC {request.logarithmic}"
        ]
        
        for cmd in commands:
            result = self.send_command(cmd)
            success, msg = self.parse_response(result)
            if not success:
                response.success = False
                response.message = f"Failed to set gain: {msg}"
                return response
        
        response.success = True
        response.message = f"{side} gain updated successfully"
        return response
    
    def set_power_callback(self, request, response):
        """Handle set_power service request."""
        side = request.side.upper()
        if side not in ["PORT", "STARBOARD"]:
            response.success = False
            response.message = "Invalid side. Must be 'port' or 'starboard'"
            return response
        
        self.logger.info(f"Setting {side} transmit power to {request.power}")
        
        command = f"SET {side} TRANSMIT POWER {request.power}"
        result = self.send_command(command)
        
        response.success, response.message = self.parse_response(result)
        return response
    
    def set_sound_velocity_callback(self, request, response):
        """Handle set_sound_velocity service request."""
        self.logger.info(f"Setting sound velocity to {request.sound_velocity} m/s")
        
        command = f"SET SOUND VELOCITY {request.sound_velocity}"
        result = self.send_command(command)
        
        response.success, response.message = self.parse_response(result)
        return response
    
    def set_trigger_callback(self, request, response):
        """Handle set_trigger_mode service request."""
        source = request.trigger_source.upper()
        self.logger.info(f"Setting trigger mode to {source}")
        
        if source == "CONTINUOUS":
            command = f"SET TRIGGER {source} {request.duty_cycle}"
        else:
            command = f"SET TRIGGER {source}"
        
        result = self.send_command(command)
        
        response.success, response.message = self.parse_response(result)
        return response
    
    def get_settings_callback(self, request, response):
        """Handle get_settings service request."""
        self.logger.info("Querying current sonar settings")
        
        # Query sonar for current settings
        # This will require sending GET/QUERY commands per the control interface
        
        result = self.send_command("GET SETTINGS")
        
        if result is None:
            response.success = False
            response.message = "Failed to query settings"
            return response
        
        # Parse response and populate SonarSettings message
        # This parsing will depend on the exact response format from the sonar
        # For now, return success with raw message
        
        response.success = True
        response.message = result
        response.settings = SonarSettings()  # TODO: Parse actual settings
        
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
