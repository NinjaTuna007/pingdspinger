#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, scrolledtext
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from pingdsp_msg.srv import SetSonarRange, SetSonarGain, SetSoundVelocity
import threading
from datetime import datetime

class SonarControlGUI:
    def __init__(self, node):
        self.node = node
        self.window = tk.Tk()
        self.window.title("Sonar Control")
        self.window.geometry("500x600")
        
        # Range control
        range_frame = ttk.LabelFrame(self.window, text="Range Control", padding=10)
        range_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(range_frame, text="Range (m):").grid(row=0, column=0, sticky="w", pady=5)
        self.range_var = tk.DoubleVar(value=50.0)
        self.range_spin = ttk.Spinbox(range_frame, from_=1.0, to=300.0, textvariable=self.range_var, width=15)
        self.range_spin.grid(row=0, column=1, pady=5)
        ttk.Button(range_frame, text="Set Range", command=self.set_range).grid(row=0, column=2, padx=5, pady=5)
        
        # Gain control
        gain_frame = ttk.LabelFrame(self.window, text="Gain Control", padding=10)
        gain_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(gain_frame, text="Gain (dB):").grid(row=0, column=0, sticky="w", pady=5)
        self.gain_var = tk.DoubleVar(value=20.0)
        self.gain_spin = ttk.Spinbox(gain_frame, from_=0.0, to=40.0, textvariable=self.gain_var, width=15)
        self.gain_spin.grid(row=0, column=1, pady=5)
        ttk.Button(gain_frame, text="Set Gain", command=self.set_gain).grid(row=0, column=2, padx=5, pady=5)
        
        # Sound velocity control
        sv_frame = ttk.LabelFrame(self.window, text="Sound Velocity", padding=10)
        sv_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(sv_frame, text="Velocity (m/s):").grid(row=0, column=0, sticky="w", pady=5)
        self.sv_var = tk.DoubleVar(value=1500.0)
        self.sv_spin = ttk.Spinbox(sv_frame, from_=1300.0, to=1700.0, textvariable=self.sv_var, width=15)
        self.sv_spin.grid(row=0, column=1, pady=5)
        ttk.Button(sv_frame, text="Set Velocity", command=self.set_sound_velocity).grid(row=0, column=2, padx=5, pady=5)
        
        # Ping button
        ping_frame = ttk.Frame(self.window, padding=10)
        ping_frame.pack(fill="x", padx=10, pady=5)
        ttk.Button(ping_frame, text="PING", command=self.ping, width=20).pack()
        
        # Status label
        self.status_var = tk.StringVar(value="Ready")
        status_frame = ttk.Frame(self.window, padding=10)
        status_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(status_frame, textvariable=self.status_var, relief="sunken", anchor="w").pack(fill="x")
        
        # Console log
        console_frame = ttk.LabelFrame(self.window, text="Console Log", padding=10)
        console_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.console = scrolledtext.ScrolledText(console_frame, height=10, state='disabled', wrap='word')
        self.console.pack(fill="both", expand=True)
        
        # Clear log button
        ttk.Button(console_frame, text="Clear Log", command=self.clear_log).pack(pady=5)
        
        self.log("GUI initialized. Ready to control sonar.")
    
    def log(self, message):
        """Add a timestamped message to the console log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.console.config(state='normal')
        self.console.insert('end', f"[{timestamp}] {message}\n")
        self.console.see('end')
        self.console.config(state='disabled')
        self.node.get_logger().info(message)
    
    def clear_log(self):
        """Clear the console log"""
        self.console.config(state='normal')
        self.console.delete('1.0', 'end')
        self.console.config(state='disabled')
    
    def set_range(self):
        range_val = self.range_var.get()
        self.status_var.set("Setting range...")
        self.log(f"Calling /sonar/set_range with range={range_val}")
        self.window.update()
        
        client = self.node.create_client(SetSonarRange, '/sonar/set_range')
        if not client.wait_for_service(timeout_sec=1.0):
            error_msg = "ERROR: set_range service unavailable"
            self.status_var.set(error_msg)
            self.log(error_msg)
            return
        
        req = SetSonarRange.Request()
        req.range = range_val
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            success_msg = f"Range set to {range_val} m"
            self.status_var.set(success_msg)
            self.log(f"✓ SUCCESS: {success_msg}")
            self.log(f"  Response: {future.result().message}")
        else:
            msg = future.result().message if future.result() else "No response"
            error_msg = f"Failed to set range: {msg}"
            self.status_var.set(error_msg)
            self.log(f"✗ FAILED: {error_msg}")
    
    def set_gain(self):
        gain_val = self.gain_var.get()
        self.status_var.set("Setting gain...")
        self.log(f"Calling /sonar/set_gain with gain={gain_val}")
        self.window.update()
        
        client = self.node.create_client(SetSonarGain, '/sonar/set_gain')
        if not client.wait_for_service(timeout_sec=1.0):
            error_msg = "ERROR: set_gain service unavailable"
            self.status_var.set(error_msg)
            self.log(error_msg)
            return
        
        req = SetSonarGain.Request()
        req.gain = gain_val
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            success_msg = f"Gain set to {gain_val} dB"
            self.status_var.set(success_msg)
            self.log(f"✓ SUCCESS: {success_msg}")
            self.log(f"  Response: {future.result().message}")
        else:
            msg = future.result().message if future.result() else "No response"
            error_msg = f"Failed to set gain: {msg}"
            self.status_var.set(error_msg)
            self.log(f"✗ FAILED: {error_msg}")
    
    def set_sound_velocity(self):
        sv_val = self.sv_var.get()
        self.status_var.set("Setting sound velocity...")
        self.log(f"Calling /sonar/set_sound_velocity with sound_velocity={sv_val}")
        self.window.update()
        
        client = self.node.create_client(SetSoundVelocity, '/sonar/set_sound_velocity')
        if not client.wait_for_service(timeout_sec=1.0):
            error_msg = "ERROR: set_sound_velocity service unavailable"
            self.status_var.set(error_msg)
            self.log(error_msg)
            return
        
        req = SetSoundVelocity.Request()
        req.sound_velocity = sv_val
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            success_msg = f"Sound velocity set to {sv_val} m/s"
            self.status_var.set(success_msg)
            self.log(f"✓ SUCCESS: {success_msg}")
            self.log(f"  Response: {future.result().message}")
        else:
            msg = future.result().message if future.result() else "No response"
            error_msg = f"Failed to set sound velocity: {msg}"
            self.status_var.set(error_msg)
            self.log(f"✗ FAILED: {error_msg}")
    
    def ping(self):
        self.status_var.set("Sending ping...")
        self.log("Calling /sonar/ping")
        self.window.update()
        
        client = self.node.create_client(Trigger, '/sonar/ping')
        if not client.wait_for_service(timeout_sec=1.0):
            error_msg = "ERROR: ping service unavailable"
            self.status_var.set(error_msg)
            self.log(error_msg)
            return
        
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            success_msg = "Ping sent successfully"
            self.status_var.set(success_msg)
            self.log(f"✓ SUCCESS: {success_msg}")
            self.log(f"  Response: {future.result().message}")
        else:
            msg = future.result().message if future.result() else "No response"
            error_msg = f"Failed to ping: {msg}"
            self.status_var.set(error_msg)
            self.log(f"✗ FAILED: {error_msg}")
    
    def run(self):
        self.window.mainloop()

def main():
    rclpy.init()
    node = rclpy.create_node('sonar_control_gui')
    
    # Spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    gui = SonarControlGUI(node)
    gui.run()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
