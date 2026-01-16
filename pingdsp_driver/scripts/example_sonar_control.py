#!/usr/bin/env python3
"""
Example script demonstrating sonar control via ROS 2 services.

This script shows how to programmatically control the 3DSS-DX sonar
settings using the service-based interface.
"""

import rclpy
from rclpy.node import Node
from pingdsp_msg.srv import (
    SetSonarRange,
    SetSonarGain,
    SetSonarPower,
    SetSoundVelocity,
    GetSonarSettings,
    SetTriggerMode
)
import time


class SonarControlExample(Node):
    """Example node showing sonar control service usage."""
    
    def __init__(self):
        super().__init__('sonar_control_example')
        
        # Create service clients
        self.range_client = self.create_client(SetSonarRange, '/sonar/set_range')
        self.gain_client = self.create_client(SetSonarGain, '/sonar/set_gain')
        self.power_client = self.create_client(SetSonarPower, '/sonar/set_power')
        self.velocity_client = self.create_client(SetSoundVelocity, '/sonar/set_sound_velocity')
        self.settings_client = self.create_client(GetSonarSettings, '/sonar/get_settings')
        self.trigger_client = self.create_client(SetTriggerMode, '/sonar/set_trigger_mode')
        
        self.get_logger().info("Waiting for services...")
        self.wait_for_services()
        self.get_logger().info("All services available!")
    
    def wait_for_services(self, timeout=10.0):
        """Wait for all services to become available."""
        clients = [
            self.range_client,
            self.gain_client,
            self.power_client,
            self.velocity_client,
            self.settings_client,
            self.trigger_client
        ]
        
        for client in clients:
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f"Service {client.srv_name} not available!")
                raise RuntimeError(f"Service {client.srv_name} not available")
    
    def set_range(self, range_meters):
        """Set sonar range."""
        request = SetSonarRange.Request()
        request.range = range_meters
        
        self.get_logger().info(f"Setting range to {range_meters}m...")
        future = self.range_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        if result.success:
            self.get_logger().info(f"✓ Range set successfully: {result.message}")
        else:
            self.get_logger().error(f"✗ Failed to set range: {result.message}")
        
        return result.success
    
    def set_gain(self, side, constant, linear, logarithmic):
        """Set TVG gain parameters."""
        request = SetSonarGain.Request()
        request.side = side
        request.constant = constant
        request.linear = linear
        request.logarithmic = logarithmic
        
        self.get_logger().info(f"Setting {side} gain (C={constant}, L={linear}, Log={logarithmic})...")
        future = self.gain_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        if result.success:
            self.get_logger().info(f"✓ Gain set successfully: {result.message}")
        else:
            self.get_logger().error(f"✗ Failed to set gain: {result.message}")
        
        return result.success
    
    def set_power(self, side, power):
        """Set transmit power."""
        request = SetSonarPower.Request()
        request.side = side
        request.power = power
        
        self.get_logger().info(f"Setting {side} power to {power}...")
        future = self.power_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        if result.success:
            self.get_logger().info(f"✓ Power set successfully: {result.message}")
        else:
            self.get_logger().error(f"✗ Failed to set power: {result.message}")
        
        return result.success
    
    def set_sound_velocity(self, velocity):
        """Set sound velocity."""
        request = SetSoundVelocity.Request()
        request.sound_velocity = velocity
        
        self.get_logger().info(f"Setting sound velocity to {velocity} m/s...")
        future = self.velocity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        if result.success:
            self.get_logger().info(f"✓ Sound velocity set successfully: {result.message}")
        else:
            self.get_logger().error(f"✗ Failed to set sound velocity: {result.message}")
        
        return result.success
    
    def get_settings(self):
        """Query current sonar settings."""
        request = GetSonarSettings.Request()
        
        self.get_logger().info("Querying current settings...")
        future = self.settings_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        if result.success:
            self.get_logger().info(f"✓ Current settings: {result.message}")
        else:
            self.get_logger().error(f"✗ Failed to get settings: {result.message}")
        
        return result.success
    
    def set_trigger_mode(self, source, duty_cycle=0.0):
        """Set trigger mode."""
        request = SetTriggerMode.Request()
        request.trigger_source = source
        request.duty_cycle = duty_cycle
        
        self.get_logger().info(f"Setting trigger mode to {source}...")
        future = self.trigger_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        result = future.result()
        if result.success:
            self.get_logger().info(f"✓ Trigger mode set successfully: {result.message}")
        else:
            self.get_logger().error(f"✗ Failed to set trigger mode: {result.message}")
        
        return result.success


def main():
    """Run example sonar control sequence."""
    rclpy.init()
    
    try:
        controller = SonarControlExample()
        
        print("\n" + "="*60)
        print("  3DSS-DX Sonar Control Example")
        print("="*60 + "\n")
        
        # Example 1: Set range to 50 meters
        print("\n--- Example 1: Set Range ---")
        controller.set_range(50.0)
        time.sleep(0.5)
        
        # Example 2: Configure gain for port side
        print("\n--- Example 2: Set Port Gain ---")
        controller.set_gain('port', constant=10.0, linear=0.5, logarithmic=20.0)
        time.sleep(0.5)
        
        # Example 3: Set transmit power
        print("\n--- Example 3: Set Starboard Power ---")
        controller.set_power('starboard', power=75)
        time.sleep(0.5)
        
        # Example 4: Set sound velocity
        print("\n--- Example 4: Set Sound Velocity ---")
        controller.set_sound_velocity(1500.0)
        time.sleep(0.5)
        
        # Example 5: Set trigger mode to continuous
        print("\n--- Example 5: Set Trigger Mode ---")
        controller.set_trigger_mode('continuous', duty_cycle=0.8)
        time.sleep(0.5)
        
        # Example 6: Query current settings
        print("\n--- Example 6: Get Current Settings ---")
        controller.get_settings()
        
        print("\n" + "="*60)
        print("  All examples completed!")
        print("="*60 + "\n")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
