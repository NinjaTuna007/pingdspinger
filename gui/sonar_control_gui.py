#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
import rclpy
from rclpy.node import Node
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
import threading
from datetime import datetime

class SonarControlGUI:
    def __init__(self, node):
        self.node = node
        self.window = tk.Tk()
        self.window.title("3DSS-DX Sonar Control")
        self.window.geometry("900x700")
        
        # Status label at top
        self.status_var = tk.StringVar(value="Ready")
        status_frame = ttk.Frame(self.window, padding=5)
        status_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(status_frame, textvariable=self.status_var, relief="sunken", anchor="w").pack(fill="x")
        
        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.window)
        self.notebook.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Create tabs
        self.create_app_tab()
        self.create_sonar_tab()
        self.create_acquisition_tab()
        self.create_transmit_tab()
        self.create_processing_tab()
        self.create_sidescan3d_tab()
        self.create_bathymetry_tab()
        self.create_sound_velocity_tab()
        self.create_file_tab()
        self.create_record_tab()
        self.create_baud_tab()
        
        # Console log at bottom
        console_frame = ttk.LabelFrame(self.window, text="Console Log", padding=5)
        console_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.console = scrolledtext.ScrolledText(console_frame, height=8, state='disabled', wrap='word')
        self.console.pack(fill="both", expand=True)
        
        ttk.Button(console_frame, text="Clear Log", command=self.clear_log).pack(pady=2)
        
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
    
    # ========================================================================
    # Tab Creation Methods
    # ========================================================================
    
    def create_app_tab(self):
        """Create application control tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="App Control")
        
        # App mode control
        mode_frame = ttk.LabelFrame(tab, text="Application Mode", padding=10)
        mode_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(mode_frame, text="Mode:").grid(row=0, column=0, sticky="w", pady=5)
        self.app_mode_var = tk.StringVar(value="sonar")
        mode_combo = ttk.Combobox(mode_frame, textvariable=self.app_mode_var, 
                                  values=["sonar", "fileprocess", "fileplay"], state="readonly", width=15)
        mode_combo.grid(row=0, column=1, pady=5, padx=5)
        
        ttk.Button(mode_frame, text="Initialize", command=lambda: self.app_control("init")).grid(row=0, column=2, padx=5)
        ttk.Button(mode_frame, text="Set Mode", command=lambda: self.app_control("mode")).grid(row=0, column=3, padx=5)
        ttk.Button(mode_frame, text="Status", command=lambda: self.app_control("status")).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(mode_frame, text="Exit App", command=lambda: self.app_control("exit")).grid(row=1, column=1, padx=5, pady=5)
    
    def create_sonar_tab(self):
        """Create sonar control tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Sonar Control")
        
        # Connection control
        conn_frame = ttk.LabelFrame(tab, text="Connection", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Button(conn_frame, text="Connect", command=lambda: self.sonar_control("connect"), width=15).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(conn_frame, text="Disconnect", command=lambda: self.sonar_control("disconnect"), width=15).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(conn_frame, text="Update Time", command=lambda: self.sonar_control("updatetime"), width=15).grid(row=0, column=2, padx=5, pady=5)
        
        # Operation control
        op_frame = ttk.LabelFrame(tab, text="Operation", padding=10)
        op_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Button(op_frame, text="Run", command=lambda: self.sonar_control("run"), width=15).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(op_frame, text="Stop", command=lambda: self.sonar_control("stop"), width=15).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(op_frame, text="Status", command=lambda: self.sonar_control("status"), width=15).grid(row=0, column=2, padx=5, pady=5)
    
    def create_acquisition_tab(self):
        """Create acquisition settings tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Acquisition")
        
        settings_frame = ttk.LabelFrame(tab, text="Acquisition Settings", padding=10)
        settings_frame.pack(fill="x", padx=10, pady=5)
        
        # Range
        ttk.Label(settings_frame, text="Range (m):").grid(row=0, column=0, sticky="w", pady=5)
        self.acq_range_var = tk.StringVar(value="75")
        range_combo = ttk.Combobox(settings_frame, textvariable=self.acq_range_var,
                                   values=["15", "20", "25", "50", "75", "100", "125", "150", "200", "250"],
                                   state="readonly", width=12)
        range_combo.grid(row=0, column=1, pady=5, padx=5)
        
        # Duty cycle
        ttk.Label(settings_frame, text="Duty Cycle (%):").grid(row=0, column=2, sticky="w", pady=5, padx=(10,0))
        self.acq_duty_var = tk.StringVar(value="100")
        duty_combo = ttk.Combobox(settings_frame, textvariable=self.acq_duty_var,
                                 values=["1", "10", "25", "50", "75", "100"], state="readonly", width=12)
        duty_combo.grid(row=0, column=3, pady=5, padx=5)
        
        # Trigger
        ttk.Label(settings_frame, text="Trigger:").grid(row=1, column=0, sticky="w", pady=5)
        self.acq_trigger_var = tk.StringVar(value="continuous")
        trigger_combo = ttk.Combobox(settings_frame, textvariable=self.acq_trigger_var,
                                     values=["continuous", "external"], state="readonly", width=12)
        trigger_combo.grid(row=1, column=1, pady=5, padx=5)
        
        # Max depth
        ttk.Label(settings_frame, text="Max Depth (m):").grid(row=1, column=2, sticky="w", pady=5, padx=(10,0))
        self.acq_maxdepth_var = tk.StringVar(value="15")
        maxdepth_combo = ttk.Combobox(settings_frame, textvariable=self.acq_maxdepth_var,
                                      values=["3", "5", "7.5", "10", "15", "25", "35", "50", "75"],
                                      state="readonly", width=12)
        maxdepth_combo.grid(row=1, column=3, pady=5, padx=5)
        
        # Environment
        ttk.Label(settings_frame, text="Environment:").grid(row=2, column=0, sticky="w", pady=5)
        self.acq_env_var = tk.StringVar(value="simple")
        env_combo = ttk.Combobox(settings_frame, textvariable=self.acq_env_var,
                                values=["simple", "complex"], state="readonly", width=12)
        env_combo.grid(row=2, column=1, pady=5, padx=5)
        
        # Priority
        ttk.Label(settings_frame, text="Priority:").grid(row=2, column=2, sticky="w", pady=5, padx=(10,0))
        self.acq_priority_var = tk.StringVar(value="bathymetry")
        priority_combo = ttk.Combobox(settings_frame, textvariable=self.acq_priority_var,
                                     values=["bathymetry", "widearea", "highres"], state="readonly", width=12)
        priority_combo.grid(row=2, column=3, pady=5, padx=5)
        
        # Buttons
        btn_frame = ttk.Frame(settings_frame)
        btn_frame.grid(row=3, column=0, columnspan=4, pady=10)
        ttk.Button(btn_frame, text="Get Settings", command=lambda: self.acquisition_settings("get")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Set Settings", command=lambda: self.acquisition_settings("set")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Commit", command=self.commit_settings, style="Accent.TButton").pack(side="left", padx=5)
    
    def create_transmit_tab(self):
        """Create transmit settings tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Transmit")
        
        settings_frame = ttk.LabelFrame(tab, text="Transmit Settings", padding=10)
        settings_frame.pack(fill="x", padx=10, pady=5)
        
        # Side selection
        ttk.Label(settings_frame, text="Side:").grid(row=0, column=0, sticky="w", pady=5)
        self.tx_side_var = tk.StringVar(value="both")
        side_combo = ttk.Combobox(settings_frame, textvariable=self.tx_side_var,
                                 values=["both", "port", "starboard"], state="readonly", width=12)
        side_combo.grid(row=0, column=1, pady=5, padx=5)
        
        # Pulse
        ttk.Label(settings_frame, text="Pulse:").grid(row=0, column=2, sticky="w", pady=5, padx=(10,0))
        self.tx_pulse_var = tk.StringVar(value="nb25")
        pulse_combo = ttk.Combobox(settings_frame, textvariable=self.tx_pulse_var,
                                   values=["none", "nb10", "nb15", "nb25", "nb50", "bb80", "bb200"],
                                   state="readonly", width=12)
        pulse_combo.grid(row=0, column=3, pady=5, padx=5)
        
        # Power
        ttk.Label(settings_frame, text="Power:").grid(row=1, column=0, sticky="w", pady=5)
        self.tx_power_var = tk.StringVar(value="80")
        power_combo = ttk.Combobox(settings_frame, textvariable=self.tx_power_var,
                                   values=["1", "2", "5", "10", "20", "50", "80", "100"],
                                   state="readonly", width=12)
        power_combo.grid(row=1, column=1, pady=5, padx=5)
        
        # Beamwidth
        ttk.Label(settings_frame, text="Beamwidth:").grid(row=1, column=2, sticky="w", pady=5, padx=(10,0))
        self.tx_beamwidth_var = tk.StringVar(value="55")
        beamwidth_combo = ttk.Combobox(settings_frame, textvariable=self.tx_beamwidth_var,
                                       values=["19", "22", "30", "44", "55", "90"], state="readonly", width=12)
        beamwidth_combo.grid(row=1, column=3, pady=5, padx=5)
        
        # Angle
        ttk.Label(settings_frame, text="Angle (deg):").grid(row=2, column=0, sticky="w", pady=5)
        self.tx_angle_var = tk.StringVar(value="0")
        angle_combo = ttk.Combobox(settings_frame, textvariable=self.tx_angle_var,
                                   values=["-45", "-30", "-20", "-15", "-10", "0", "10", "15", "20", "30", "45"],
                                   state="readonly", width=12)
        angle_combo.grid(row=2, column=1, pady=5, padx=5)
        
        # Buttons
        btn_frame = ttk.Frame(settings_frame)
        btn_frame.grid(row=3, column=0, columnspan=4, pady=10)
        ttk.Button(btn_frame, text="Get Settings", command=lambda: self.transmit_settings("get")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Set Settings", command=lambda: self.transmit_settings("set")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Commit", command=self.commit_settings, style="Accent.TButton").pack(side="left", padx=5)
    
    def create_processing_tab(self):
        """Create processing settings tab (sidescan, sidescan3d, bathymetry)"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Processing")
        
        # Sidescan
        sidescan_frame = ttk.LabelFrame(tab, text="Sidescan Settings", padding=10)
        sidescan_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(sidescan_frame, text="Side:").grid(row=0, column=0, sticky="w", pady=5)
        self.ss_side_var = tk.StringVar(value="both")
        ttk.Combobox(sidescan_frame, textvariable=self.ss_side_var,
                    values=["both", "port", "starboard"], state="readonly", width=12).grid(row=0, column=1, pady=5, padx=5)
        
        ttk.Label(sidescan_frame, text="Mode:").grid(row=0, column=2, sticky="w", pady=5, padx=(10,0))
        self.ss_mode_var = tk.StringVar(value="incoherent")
        ttk.Combobox(sidescan_frame, textvariable=self.ss_mode_var,
                    values=["incoherent", "coherent"], state="readonly", width=12).grid(row=0, column=3, pady=5, padx=5)
        
        ttk.Label(sidescan_frame, text="Method:").grid(row=1, column=0, sticky="w", pady=5)
        self.ss_method_var = tk.StringVar(value="rms")
        ttk.Combobox(sidescan_frame, textvariable=self.ss_method_var,
                    values=["rms", "max"], state="readonly", width=12).grid(row=1, column=1, pady=5, padx=5)
        
        ttk.Label(sidescan_frame, text="Beams (9-bit):").grid(row=1, column=2, sticky="w", pady=5, padx=(10,0))
        self.ss_beams_var = tk.StringVar(value="001100000")
        ttk.Entry(sidescan_frame, textvariable=self.ss_beams_var, width=14).grid(row=1, column=3, pady=5, padx=5)
        
        btn_frame = ttk.Frame(sidescan_frame)
        btn_frame.grid(row=2, column=0, columnspan=4, pady=5)
        ttk.Button(btn_frame, text="Get", command=lambda: self.sidescan_settings("get")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Set", command=lambda: self.sidescan_settings("set")).pack(side="left", padx=5)
        
        # Sidescan3D and Bathymetry - moved to their own tabs
    
    def create_sidescan3d_tab(self):
        """Create sidescan 3D settings tab (Section 3.5)"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Sidescan 3D")
        
        # Create scrollable frame
        canvas = tk.Canvas(tab)
        scrollbar = ttk.Scrollbar(tab, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Side selection
        side_frame = ttk.LabelFrame(scrollable_frame, text="Side Selection", padding=10)
        side_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(side_frame, text="Apply to:").grid(row=0, column=0, sticky="w", pady=5)
        self.ss3d_side_var = tk.StringVar(value="both")
        ttk.Combobox(side_frame, textvariable=self.ss3d_side_var,
                    values=["both", "port", "stbd"], state="readonly", width=12).grid(row=0, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(side_frame, text="(image filtering applies to both sides)").grid(row=0, column=2, sticky="w", pady=5, padx=5)
        
        # Angle and Smoothing Settings
        angle_frame = ttk.LabelFrame(scrollable_frame, text="Angle & Smoothing", padding=10)
        angle_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(angle_frame, text="Angles:").grid(row=0, column=0, sticky="w", pady=5)
        self.ss3d_angles_var = tk.IntVar(value=3)
        ttk.Combobox(angle_frame, textvariable=self.ss3d_angles_var,
                    values=[1, 2, 3, 4], state="readonly", width=12).grid(row=0, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(angle_frame, text="(1, 2, 3, or 4)").grid(row=0, column=2, sticky="w", pady=5)
        
        ttk.Label(angle_frame, text="Smoothing:").grid(row=1, column=0, sticky="w", pady=5)
        self.ss3d_smoothing_var = tk.IntVar(value=20)
        ttk.Combobox(angle_frame, textvariable=self.ss3d_smoothing_var,
                    values=[0, 2, 3, 5, 10, 15, 20, 25, 30, 40, 50], state="readonly", width=12).grid(row=1, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(angle_frame, text="(0, 2, 3, 5, 10, 15, 20, 25, 30, 40, 50)").grid(row=1, column=2, sticky="w", pady=5)
        
        # Threshold and Tolerance
        thresh_frame = ttk.LabelFrame(scrollable_frame, text="Threshold & Tolerance", padding=10)
        thresh_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(thresh_frame, text="Threshold (dB):").grid(row=0, column=0, sticky="w", pady=5)
        self.ss3d_threshold_var = tk.DoubleVar(value=-100.0)
        ttk.Spinbox(thresh_frame, textvariable=self.ss3d_threshold_var, from_=-150, to=0, 
                   increment=1, width=12).grid(row=0, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(thresh_frame, text="(-150 to 0)").grid(row=0, column=2, sticky="w", pady=5)
        
        ttk.Label(thresh_frame, text="Tolerance:").grid(row=1, column=0, sticky="w", pady=5)
        self.ss3d_tolerance_var = tk.DoubleVar(value=0.1)
        ttk.Spinbox(thresh_frame, textvariable=self.ss3d_tolerance_var, from_=0.001, to=0.999, 
                   increment=0.001, width=12).grid(row=1, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(thresh_frame, text="(0.001 to 0.999)").grid(row=1, column=2, sticky="w", pady=5)
        
        # Depth and Swath Settings
        depth_frame = ttk.LabelFrame(scrollable_frame, text="Depth & Swath Filtering", padding=10)
        depth_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(depth_frame, text="Min Depth:").grid(row=0, column=0, sticky="w", pady=5)
        self.ss3d_mindepth_var = tk.DoubleVar(value=1.0)
        ttk.Spinbox(depth_frame, textvariable=self.ss3d_mindepth_var, from_=0, to=200, 
                   increment=0.1, width=12).grid(row=0, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(depth_frame, text="m").grid(row=0, column=2, sticky="w", pady=5)
        
        ttk.Label(depth_frame, text="Max Depth:").grid(row=1, column=0, sticky="w", pady=5)
        self.ss3d_maxdepth_var = tk.DoubleVar(value=200.0)
        ttk.Spinbox(depth_frame, textvariable=self.ss3d_maxdepth_var, from_=0, to=200, 
                   increment=1, width=12).grid(row=1, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(depth_frame, text="m").grid(row=1, column=2, sticky="w", pady=5)
        
        ttk.Label(depth_frame, text="Swath:").grid(row=2, column=0, sticky="w", pady=5)
        self.ss3d_swath_var = tk.DoubleVar(value=12.0)
        ttk.Spinbox(depth_frame, textvariable=self.ss3d_swath_var, from_=0, to=50, 
                   increment=0.5, width=12).grid(row=2, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(depth_frame, text="m").grid(row=2, column=2, sticky="w", pady=5)
        
        ttk.Label(depth_frame, text="Amplitude:").grid(row=3, column=0, sticky="w", pady=5)
        self.ss3d_amp_var = tk.DoubleVar(value=0.3)
        ttk.Spinbox(depth_frame, textvariable=self.ss3d_amp_var, from_=0, to=1.0, 
                   increment=0.01, width=12).grid(row=3, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(depth_frame, text="(0 to 1.0)").grid(row=3, column=2, sticky="w", pady=5)
        
        # Control buttons
        btn_frame = ttk.Frame(scrollable_frame)
        btn_frame.pack(fill="x", padx=10, pady=10)
        ttk.Button(btn_frame, text="Get Current Settings", 
                  command=lambda: self.sidescan3d_settings("get")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Set Settings", 
                  command=lambda: self.sidescan3d_settings("set")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Get Port Only", 
                  command=lambda: self.sidescan3d_settings("get", "port")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Get Stbd Only", 
                  command=lambda: self.sidescan3d_settings("get", "stbd")).pack(side="left", padx=5)
        
        # Info frame
        info_frame = ttk.LabelFrame(scrollable_frame, text="Information", padding=10)
        info_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(info_frame, text="Command: sidescan3d [options]\n"
                                  "• Angles: number of angle estimates (1-4)\n"
                                  "• Smoothing: filtering amount (0, 2, 3, 5, 10, 15, 20, 25, 30, 40, 50)\n"
                                  "• Threshold: minimum energy in dB (-150 to 0)\n"
                                  "• Tolerance: solution tolerance (0.001 to 0.999)\n"
                                  "• Image filtering applies to both sides regardless of selection\n"
                                  "• Requires commit command after changes",
                 justify="left").pack(anchor="w")
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
    
    def create_bathymetry_tab(self):
        """Create sound velocity tab (Section 3.2)"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Sound Velocity")
        
        sv_frame = ttk.LabelFrame(tab, text="Sound Velocity Control (sv command)", padding=10)
        sv_frame.pack(fill="x", padx=10, pady=5)
        
        # Bulk velocity input
        ttk.Label(sv_frame, text="Bulk Velocity (m/s):").grid(row=0, column=0, sticky="w", pady=5)
        self.sv_bulk_var = tk.DoubleVar(value=1500.0)
        ttk.Spinbox(sv_frame, textvariable=self.sv_bulk_var, from_=1300.0, to=2500.0, 
                   increment=1.0, width=15).grid(row=0, column=1, pady=5, padx=5, sticky="w")
        self.sv_bulk_check = tk.BooleanVar(value=True)
        ttk.Checkbutton(sv_frame, text="Set bulk", variable=self.sv_bulk_check).grid(row=0, column=2, pady=5, padx=5)
        
        # Face velocity input
        ttk.Label(sv_frame, text="Face Velocity (m/s):").grid(row=1, column=0, sticky="w", pady=5)
        self.sv_face_var = tk.DoubleVar(value=1500.0)
        ttk.Spinbox(sv_frame, textvariable=self.sv_face_var, from_=1300.0, to=2500.0, 
                   increment=1.0, width=15).grid(row=1, column=1, pady=5, padx=5, sticky="w")
        self.sv_face_check = tk.BooleanVar(value=False)
        ttk.Checkbutton(sv_frame, text="Set face", variable=self.sv_face_check).grid(row=1, column=2, pady=5, padx=5)
        
        # Buttons
        btn_frame = ttk.Frame(sv_frame)
        btn_frame.grid(row=2, column=0, columnspan=3, pady=10)
        ttk.Button(btn_frame, text="Get Current", command=lambda: self.sound_velocity_control("get")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Set Velocity", command=lambda: self.sound_velocity_control("set")).pack(side="left", padx=5)
        
        # Info label
        info_frame = ttk.LabelFrame(tab, text="Information", padding=10)
        info_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(info_frame, text="Command: sv [--bulk=<value> | --face=<value>]\n"
                                  "• Bulk: body sound velocity\n"
                                  "• Face: transducer face sound velocity\n"
                                  "• Range: 1300-2500 m/s\n"
                                  "• Requires commit command after changes\n\n"
                                  "Typical values:\n"
                                  "• Fresh water: ~1480 m/s\n"
                                  "• Sea water (cold): ~1450 m/s\n"
                                  "• Sea water (warm): ~1540 m/s",
                 justify="left").pack(anchor="w")
    
    def create_bathymetry_tab(self):
        """Create bathymetry settings tab (Section 3.6)"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Bathymetry")
        
        # Create scrollable frame
        canvas = tk.Canvas(tab)
        scrollbar = ttk.Scrollbar(tab, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Point Rejection frame
        point_frame = ttk.LabelFrame(scrollable_frame, text="Point Rejection", padding=10)
        point_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(point_frame, text="Min Depth:").grid(row=0, column=0, sticky="w", pady=5)
        self.bathy_mindepth_var = tk.DoubleVar(value=0.5)
        ttk.Spinbox(point_frame, textvariable=self.bathy_mindepth_var, from_=-1, to=200, 
                   increment=0.1, width=12).grid(row=0, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(point_frame, text="m (-1 to 200)").grid(row=0, column=2, sticky="w", pady=5)
        
        ttk.Label(point_frame, text="Max Depth:").grid(row=1, column=0, sticky="w", pady=5)
        self.bathy_maxdepth_var = tk.DoubleVar(value=25.0)
        ttk.Spinbox(point_frame, textvariable=self.bathy_maxdepth_var, from_=0, to=200, 
                   increment=1, width=12).grid(row=1, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(point_frame, text="m (0 to 200)").grid(row=1, column=2, sticky="w", pady=5)
        
        ttk.Label(point_frame, text="Swath:").grid(row=2, column=0, sticky="w", pady=5)
        self.bathy_swath_var = tk.DoubleVar(value=8.0)
        ttk.Spinbox(point_frame, textvariable=self.bathy_swath_var, from_=0, to=20, 
                   increment=0.5, width=12).grid(row=2, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(point_frame, text="m (0 to 20)").grid(row=2, column=2, sticky="w", pady=5)
        
        # Binning frame
        binning_frame = ttk.LabelFrame(scrollable_frame, text="Binning Settings", padding=10)
        binning_frame.pack(fill="x", padx=10, pady=5)
        
        self.bathy_binning_enable = tk.BooleanVar(value=False)
        ttk.Checkbutton(binning_frame, text="Enable Binning", 
                       variable=self.bathy_binning_enable).grid(row=0, column=0, columnspan=3, sticky="w", pady=5)
        
        ttk.Label(binning_frame, text="Mode:").grid(row=1, column=0, sticky="w", pady=5)
        self.bathy_binning_mode_var = tk.StringVar(value="equidistant")
        ttk.Entry(binning_frame, textvariable=self.bathy_binning_mode_var, width=15, 
                 state="readonly").grid(row=1, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(binning_frame, text="(must be 'equidistant')").grid(row=1, column=2, sticky="w", pady=5)
        
        ttk.Label(binning_frame, text="Bin Count:").grid(row=2, column=0, sticky="w", pady=5)
        self.bathy_binning_count_var = tk.IntVar(value=1440)
        ttk.Spinbox(binning_frame, textvariable=self.bathy_binning_count_var, from_=3, to=1440, 
                   increment=1, width=12).grid(row=2, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(binning_frame, text="(3 to 1440)").grid(row=2, column=2, sticky="w", pady=5)
        
        ttk.Label(binning_frame, text="Bin Width:").grid(row=3, column=0, sticky="w", pady=5)
        self.bathy_binning_width_var = tk.DoubleVar(value=0.2)
        ttk.Spinbox(binning_frame, textvariable=self.bathy_binning_width_var, from_=0.05, to=2.0, 
                   increment=0.05, width=12).grid(row=3, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(binning_frame, text="m (0.05 to 2.0)").grid(row=3, column=2, sticky="w", pady=5)
        
        # Bottom Track frame
        bt_frame = ttk.LabelFrame(scrollable_frame, text="Bottom Track Settings", padding=10)
        bt_frame.pack(fill="x", padx=10, pady=5)
        
        self.bathy_bt_enable = tk.BooleanVar(value=False)
        ttk.Checkbutton(bt_frame, text="Enable Bottom Track", 
                       variable=self.bathy_bt_enable).grid(row=0, column=0, columnspan=3, sticky="w", pady=5)
        
        ttk.Label(bt_frame, text="Mode:").grid(row=1, column=0, sticky="w", pady=5)
        self.bathy_bt_mode_var = tk.StringVar(value="cartesian")
        ttk.Entry(bt_frame, textvariable=self.bathy_bt_mode_var, width=15, 
                 state="readonly").grid(row=1, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(bt_frame, text="(must be 'cartesian')").grid(row=1, column=2, sticky="w", pady=5)
        
        ttk.Label(bt_frame, text="Cells:").grid(row=2, column=0, sticky="w", pady=5)
        self.bathy_bt_cells_var = tk.IntVar(value=200)
        ttk.Spinbox(bt_frame, textvariable=self.bathy_bt_cells_var, from_=3, to=256, 
                   increment=1, width=12).grid(row=2, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(bt_frame, text="(3 to 256)").grid(row=2, column=2, sticky="w", pady=5)
        
        ttk.Label(bt_frame, text="Cell Width:").grid(row=3, column=0, sticky="w", pady=5)
        self.bathy_bt_width_var = tk.DoubleVar(value=1.0)
        ttk.Spinbox(bt_frame, textvariable=self.bathy_bt_width_var, from_=0.1, to=10.0, 
                   increment=0.1, width=12).grid(row=3, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(bt_frame, text="m (0.1 to 10.0)").grid(row=3, column=2, sticky="w", pady=5)
        
        ttk.Label(bt_frame, text="Cell Height:").grid(row=4, column=0, sticky="w", pady=5)
        self.bathy_bt_height_var = tk.DoubleVar(value=0.5)
        ttk.Spinbox(bt_frame, textvariable=self.bathy_bt_height_var, from_=0.1, to=10.0, 
                   increment=0.1, width=12).grid(row=4, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(bt_frame, text="m (0.1 to 10.0)").grid(row=4, column=2, sticky="w", pady=5)
        
        ttk.Label(bt_frame, text="Height Percent:").grid(row=5, column=0, sticky="w", pady=5)
        self.bathy_bt_heightp_var = tk.IntVar(value=10)
        ttk.Spinbox(bt_frame, textvariable=self.bathy_bt_heightp_var, from_=0, to=25, 
                   increment=1, width=12).grid(row=5, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(bt_frame, text="% (0 to 25)").grid(row=5, column=2, sticky="w", pady=5)
        
        ttk.Label(bt_frame, text="Alpha:").grid(row=6, column=0, sticky="w", pady=5)
        self.bathy_bt_alpha_var = tk.DoubleVar(value=0.5)
        ttk.Spinbox(bt_frame, textvariable=self.bathy_bt_alpha_var, from_=0.01, to=0.99, 
                   increment=0.01, width=12).grid(row=6, column=1, pady=5, padx=5, sticky="w")
        ttk.Label(bt_frame, text="(0.01 to 0.99)").grid(row=6, column=2, sticky="w", pady=5)
        
        # Control buttons
        btn_frame = ttk.Frame(scrollable_frame)
        btn_frame.pack(fill="x", padx=10, pady=10)
        ttk.Button(btn_frame, text="Get Current Settings", 
                  command=lambda: self.bathymetry_settings("get")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Set Settings", 
                  command=lambda: self.bathymetry_settings("set")).pack(side="left", padx=5)
        
        # Info frame
        info_frame = ttk.LabelFrame(scrollable_frame, text="Information", padding=10)
        info_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(info_frame, text="Command: bathymetry [options]\n"
                                  "• Point rejection controls depth filtering\n"
                                  "• Binning: equidistant mode for bin creation\n"
                                  "• Bottom Track: cartesian mode for tracking\n"
                                  "• Requires commit command after changes",
                 justify="left").pack(anchor="w")
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
    
    def create_sound_velocity_tab(self):
        """Create sound velocity tab (Section 3.2)"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Sound Velocity")
        
        sv_frame = ttk.LabelFrame(tab, text="Sound Velocity Control (sv command)", padding=10)
        sv_frame.pack(fill="x", padx=10, pady=5)
        
        # Bulk velocity input
        ttk.Label(sv_frame, text="Bulk Velocity (m/s):").grid(row=0, column=0, sticky="w", pady=5)
        self.sv_bulk_var = tk.DoubleVar(value=1500.0)
        ttk.Spinbox(sv_frame, textvariable=self.sv_bulk_var, from_=1300.0, to=2500.0, 
                   increment=1.0, width=15).grid(row=0, column=1, pady=5, padx=5, sticky="w")
        self.sv_bulk_check = tk.BooleanVar(value=True)
        ttk.Checkbutton(sv_frame, text="Set bulk", variable=self.sv_bulk_check).grid(row=0, column=2, pady=5, padx=5)
        
        # Face velocity input
        ttk.Label(sv_frame, text="Face Velocity (m/s):").grid(row=1, column=0, sticky="w", pady=5)
        self.sv_face_var = tk.DoubleVar(value=1500.0)
        ttk.Spinbox(sv_frame, textvariable=self.sv_face_var, from_=1300.0, to=2500.0, 
                   increment=1.0, width=15).grid(row=1, column=1, pady=5, padx=5, sticky="w")
        self.sv_face_check = tk.BooleanVar(value=False)
        ttk.Checkbutton(sv_frame, text="Set face", variable=self.sv_face_check).grid(row=1, column=2, pady=5, padx=5)
        
        # Buttons
        btn_frame = ttk.Frame(sv_frame)
        btn_frame.grid(row=2, column=0, columnspan=3, pady=10)
        ttk.Button(btn_frame, text="Get Current", command=lambda: self.sound_velocity_control("get")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Set Velocity", command=lambda: self.sound_velocity_control("set")).pack(side="left", padx=5)
        
        # Info label
        info_frame = ttk.LabelFrame(tab, text="Information", padding=10)
        info_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(info_frame, text="Command: sv [--bulk=<value> | --face=<value>]\n"
                                  "• Bulk: body sound velocity\n"
                                  "• Face: transducer face sound velocity\n"
                                  "• Range: 1300-2500 m/s\n"
                                  "• Requires commit command after changes\n\n"
                                  "Typical values:\n"
                                  "• Fresh water: ~1480 m/s\n"
                                  "• Sea water (cold): ~1450 m/s\n"
                                  "• Sea water (warm): ~1540 m/s",
                 justify="left").pack(anchor="w")
    
    def create_file_tab(self):
        """Create file control tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="File")
        
        file_frame = ttk.LabelFrame(tab, text="File Control", padding=10)
        file_frame.pack(fill="x", padx=10, pady=5)
        
        # File path
        ttk.Label(file_frame, text="File:").grid(row=0, column=0, sticky="w", pady=5)
        self.file_path_var = tk.StringVar()
        ttk.Entry(file_frame, textvariable=self.file_path_var, width=40).grid(row=0, column=1, columnspan=2, pady=5, padx=5)
        ttk.Button(file_frame, text="Browse", command=self.browse_file).grid(row=0, column=3, pady=5, padx=5)
        
        # Speed
        ttk.Label(file_frame, text="Speed:").grid(row=1, column=0, sticky="w", pady=5)
        self.file_speed_var = tk.StringVar(value="1.0")
        ttk.Combobox(file_frame, textvariable=self.file_speed_var,
                    values=["0.125", "0.25", "0.5", "1.0", "2.0", "5.0", "10.0", "20.0"],
                    state="readonly", width=12).grid(row=1, column=1, pady=5, padx=5)
        
        # Control buttons
        btn_frame = ttk.Frame(file_frame)
        btn_frame.grid(row=2, column=0, columnspan=4, pady=10)
        ttk.Button(btn_frame, text="Open", command=lambda: self.file_control("open")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Play", command=lambda: self.file_control("play")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Stop", command=lambda: self.file_control("stop")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Close", command=lambda: self.file_control("close")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Status", command=lambda: self.file_control("status")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Set Speed", command=lambda: self.file_control("speed")).pack(side="left", padx=5)
    
    def create_record_tab(self):
        """Create record control tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Record")
        
        record_frame = ttk.LabelFrame(tab, text="Recording Control", padding=10)
        record_frame.pack(fill="x", padx=10, pady=5)
        
        # File path
        ttk.Label(record_frame, text="File:").grid(row=0, column=0, sticky="w", pady=5)
        self.record_path_var = tk.StringVar()
        ttk.Entry(record_frame, textvariable=self.record_path_var, width=40).grid(row=0, column=1, columnspan=2, pady=5, padx=5)
        ttk.Button(record_frame, text="Browse", command=self.browse_record_file).grid(row=0, column=3, pady=5, padx=5)
        
        # Options
        ttk.Label(record_frame, text="Mode:").grid(row=1, column=0, sticky="w", pady=5)
        self.record_mode_var = tk.IntVar(value=0)
        ttk.Combobox(record_frame, textvariable=self.record_mode_var,
                    values=["0 (default)", "1 (playback only)", "2 (processed + playback)"],
                    state="readonly", width=30).grid(row=1, column=1, pady=5, padx=5)
        
        self.record_overwrite_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(record_frame, text="Allow Overwrite", variable=self.record_overwrite_var).grid(row=1, column=2, pady=5, padx=5)
        
        # Control buttons
        btn_frame = ttk.Frame(record_frame)
        btn_frame.grid(row=2, column=0, columnspan=4, pady=10)
        ttk.Button(btn_frame, text="Start Recording", command=lambda: self.record_control("start")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Stop Recording", command=lambda: self.record_control("stop")).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Status", command=lambda: self.record_control("status")).pack(side="left", padx=5)
    
    def create_baud_tab(self):
        """Create baud settings tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Baud")
        
        baud_frame = ttk.LabelFrame(tab, text="Baud Rate Settings", padding=10)
        baud_frame.pack(fill="x", padx=10, pady=5)
        
        # GPS baud
        ttk.Label(baud_frame, text="GPS Baud:").grid(row=0, column=0, sticky="w", pady=5)
        self.gps_baud_var = tk.StringVar(value="57600")
        ttk.Combobox(baud_frame, textvariable=self.gps_baud_var,
                    values=["9600", "19200", "38400", "57600", "115200", "230400"],
                    state="readonly", width=12).grid(row=0, column=1, pady=5, padx=5)
        ttk.Button(baud_frame, text="Set GPS Baud", command=lambda: self.baud_settings("gps")).grid(row=0, column=2, pady=5, padx=5)
        
        # MRU baud
        ttk.Label(baud_frame, text="MRU Baud:").grid(row=1, column=0, sticky="w", pady=5)
        self.mru_baud_var = tk.StringVar(value="38400")
        ttk.Combobox(baud_frame, textvariable=self.mru_baud_var,
                    values=["9600", "19200", "38400", "57600", "115200", "230400"],
                    state="readonly", width=12).grid(row=1, column=1, pady=5, padx=5)
        ttk.Button(baud_frame, text="Set MRU Baud", command=lambda: self.baud_settings("mru")).grid(row=1, column=2, pady=5, padx=5)
        
        # Get current settings
        ttk.Button(baud_frame, text="Get Current Baud Rates", command=lambda: self.baud_settings("get")).grid(row=2, column=0, columnspan=3, pady=10)
        
        # Warning label
        ttk.Label(baud_frame, text="⚠ Warning: After setting baud rates, disconnect and power cycle the sonar",
                 foreground="red").grid(row=3, column=0, columnspan=3, pady=5)
    
    # ========================================================================
    # Service Call Methods
    # ========================================================================
    
    def sound_velocity_control(self, command):
        """Call sound velocity service"""
        self.status_var.set(f"Sound velocity: {command}...")
        self.log(f"Calling /sonar/sound_velocity with command={command}")
        self.window.update()
        
        client = self.node.create_client(SoundVelocity, '/sonar/sound_velocity')
        if not client.wait_for_service(timeout_sec=1.0):
            self.log("ERROR: sound_velocity service unavailable")
            return
        
        req = SoundVelocity.Request()
        req.command = command
        if command == "set":
            # Set bulk and/or face based on checkboxes
            req.bulk_velocity = float(self.sv_bulk_var.get()) if self.sv_bulk_check.get() else 0.0
            req.face_velocity = float(self.sv_face_var.get()) if self.sv_face_check.get() else 0.0
            
            if req.bulk_velocity == 0.0 and req.face_velocity == 0.0:
                self.log("ERROR: Must check at least one of 'Set bulk' or 'Set face'")
                self.status_var.set("Error: No velocity selected")
                return
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            msg = f"Sound velocity {command} successful"
            if command == "get":
                if future.result().current_bulk > 0:
                    msg += f" (bulk={future.result().current_bulk:.1f} m/s"
                    self.sv_bulk_var.set(future.result().current_bulk)
                if future.result().current_face > 0:
                    msg += f", face={future.result().current_face:.1f} m/s)"
                    self.sv_face_var.set(future.result().current_face)
                elif future.result().current_bulk > 0:
                    msg += ")"
            self.status_var.set(msg)
            self.log(f"✓ SUCCESS: {msg}")
            self.log(f"  Response: {future.result().message}")
        else:
            msg = future.result().message if future.result() else "No response"
            self.status_var.set(f"Failed: {msg}")
            self.log(f"✗ FAILED: {msg}")
            """Call app control service"""
        self.status_var.set(f"App control: {command}...")
        self.log(f"Calling /sonar/app_control with command={command}")
        self.window.update()
        
        client = self.node.create_client(AppControl, '/sonar/app_control')
        if not client.wait_for_service(timeout_sec=1.0):
            self.log("ERROR: app_control service unavailable")
            return
        
        req = AppControl.Request()
        req.command = command
        if command in ["init", "mode"]:
            req.mode = self.app_mode_var.get()
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            msg = f"{command.capitalize()} successful"
            if future.result().current_mode:
                msg += f" (mode={future.result().current_mode})"
            self.status_var.set(msg)
            self.log(f"✓ SUCCESS: {msg}")
            self.log(f"  Response: {future.result().message}")
        else:
            msg = future.result().message if future.result() else "No response"
            self.status_var.set(f"Failed: {msg}")
            self.log(f"✗ FAILED: {msg}")
    
    def app_control(self, command):
        """Call app control service"""
        self.status_var.set(f"App: {command}...")
        self.log(f"Calling /sonar/app_control with command={command}")
        self.window.update()
        
        client = self.node.create_client(AppControl, '/sonar/app_control')
        if not client.wait_for_service(timeout_sec=1.0):
            self.log("ERROR: app_control service unavailable")
            return
        
        req = AppControl.Request()
        req.command = command
        if command == "mode":
            req.mode = self.app_mode_var.get()
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            msg = f"App {command} successful"
            if command == "status" and future.result().current_mode:
                msg += f" (mode={future.result().current_mode})"
            self.status_var.set(msg)
            self.log(f"✓ SUCCESS: {msg}")
            self.log(f"  Response: {future.result().message}")
        else:
            msg = future.result().message if future.result() else "No response"
            self.status_var.set(f"Failed: {msg}")
            self.log(f"✗ FAILED: {msg}")
    
    def sonar_control(self, command):
        """Call sonar control service"""
        self.status_var.set(f"Sonar: {command}...")
        self.log(f"Calling /sonar/control with command={command}")
        self.window.update()
        
        client = self.node.create_client(SonarControl, '/sonar/control')
        if not client.wait_for_service(timeout_sec=1.0):
            self.log("ERROR: sonar control service unavailable")
            return
        
        req = SonarControl.Request()
        req.command = command
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            msg = f"{command.capitalize()} successful"
            if command == "status" and future.result().sonar_id:
                msg += f" (ID={future.result().sonar_id}, pings={future.result().pings}, rate={future.result().ratehz:.1f}Hz)"
            self.status_var.set(msg)
            self.log(f"✓ SUCCESS: {msg}")
            self.log(f"  Response: {future.result().message}")
        else:
            msg = future.result().message if future.result() else "No response"
            self.status_var.set(f"Failed: {msg}")
            self.log(f"✗ FAILED: {msg}")
    
    def acquisition_settings(self, command):
        """Call acquisition settings service"""
        self.status_var.set(f"Acquisition: {command}...")
        self.log(f"Calling /sonar/acquisition with command={command}")
        self.window.update()
        
        client = self.node.create_client(AcquisitionSettings, '/sonar/acquisition')
        if not client.wait_for_service(timeout_sec=1.0):
            self.log("ERROR: acquisition service unavailable")
            return
        
        req = AcquisitionSettings.Request()
        req.command = command
        if command == "set":
            req.range = int(self.acq_range_var.get())
            req.dutycycle = int(self.acq_duty_var.get())
            req.trigger = self.acq_trigger_var.get()
            req.maxdepth = float(self.acq_maxdepth_var.get())
            req.env = self.acq_env_var.get()
            req.priority = self.acq_priority_var.get()
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.status_var.set("Acquisition settings updated")
            self.log(f"✓ SUCCESS: Acquisition settings {command}")
            self.log(f"  Response: {future.result().message}")
            
            # Parse and update GUI values for get command
            if command == "get":
                self._parse_acquisition_response(future.result().message)
        else:
            msg = future.result().message if future.result() else "No response"
            self.status_var.set(f"Failed: {msg}")
            self.log(f"✗ FAILED: {msg}")
    
    def _parse_acquisition_response(self, response):
        """Parse acquisition response and update GUI values"""
        import re
        if not response:
            return
        
        # Response format: "okay (dutycycle=100 range=75 trigger=continuous maxdepth=15 env=simple priority=bathymetry)"
        range_match = re.search(r'range=(\d+)', response)
        duty_match = re.search(r'dutycycle=(\d+)', response)
        trigger_match = re.search(r'trigger=(\w+)', response)
        maxdepth_match = re.search(r'maxdepth=([\d.]+)', response)
        env_match = re.search(r'env=(\w+)', response)
        priority_match = re.search(r'priority=(\w+)', response)
        
        if range_match:
            self.acq_range_var.set(range_match.group(1))
        if duty_match:
            self.acq_duty_var.set(duty_match.group(1))
        if trigger_match:
            self.acq_trigger_var.set(trigger_match.group(1))
        if maxdepth_match:
            self.acq_maxdepth_var.set(maxdepth_match.group(1))
        if env_match:
            self.acq_env_var.set(env_match.group(1))
        if priority_match:
            self.acq_priority_var.set(priority_match.group(1))
        
        self.log("  GUI values updated from current settings")
    
    def transmit_settings(self, command):
        """Call transmit settings service"""
        self.status_var.set(f"Transmit: {command}...")
        self.log(f"Calling /sonar/transmit with command={command}")
        self.window.update()
        
        client = self.node.create_client(TransmitSettings, '/sonar/transmit')
        if not client.wait_for_service(timeout_sec=1.0):
            self.log("ERROR: transmit service unavailable")
            return
        
        req = TransmitSettings.Request()
        req.command = command
        req.side = self.tx_side_var.get()
        if command == "set":
            req.pulse = self.tx_pulse_var.get()
            req.power = int(self.tx_power_var.get())
            req.beamwidth = int(self.tx_beamwidth_var.get())
            req.angle = int(self.tx_angle_var.get())
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.status_var.set("Transmit settings updated")
            self.log(f"✓ SUCCESS: Transmit settings {command}")
            self.log(f"  Response: {future.result().message}")
            
            # Parse and update GUI values for get command
            if command == "get":
                self._parse_transmit_response(future.result().message, req.side)
        else:
            msg = future.result().message if future.result() else "No response"
            self.status_var.set(f"Failed: {msg}")
            self.log(f"✗ FAILED: {msg}")
    
    def _parse_transmit_response(self, response, side):
        """Parse transmit response and update GUI values"""
        import re
        if not response:
            return
        
        # Response format: "okay port(pulse=bb200 power=80 beamwidth=55 angle=0) stbd(pulse=nb25 power=80 beamwidth=55 angle=0)"
        # If side was specified, only that side is returned
        if side == "port" or side == "both":
            port_match = re.search(r'port\(([^)]+)\)', response)
            if port_match:
                self._parse_transmit_side_values(port_match.group(1))
        elif side == "starboard" or side == "stbd":
            stbd_match = re.search(r'stbd\(([^)]+)\)', response)
            if stbd_match:
                self._parse_transmit_side_values(stbd_match.group(1))
        
        # If both sides returned and GUI side is "both", parse port values
        if side == "both":
            port_match = re.search(r'port\(([^)]+)\)', response)
            if port_match:
                self._parse_transmit_side_values(port_match.group(1))
        
        self.log("  GUI values updated from current settings")
    
    def _parse_transmit_side_values(self, values_str):
        """Parse transmit values from a side's response"""
        import re
        pulse_match = re.search(r'pulse=(\w+)', values_str)
        power_match = re.search(r'power=(\d+)', values_str)
        beamwidth_match = re.search(r'beamwidth=(\d+)', values_str)
        angle_match = re.search(r'angle=(-?\d+)', values_str)
        
        if pulse_match:
            self.tx_pulse_var.set(pulse_match.group(1))
        if power_match:
            self.tx_power_var.set(power_match.group(1))
        if beamwidth_match:
            self.tx_beamwidth_var.set(beamwidth_match.group(1))
        if angle_match:
            self.tx_angle_var.set(angle_match.group(1))
    
    def sidescan_settings(self, command):
        """Call sidescan settings service"""
        self.status_var.set(f"Sidescan: {command}...")
        self.log(f"Calling /sonar/sidescan with command={command}")
        self.window.update()
        
        client = self.node.create_client(SidescanSettings, '/sonar/sidescan')
        if not client.wait_for_service(timeout_sec=1.0):
            self.log("ERROR: sidescan service unavailable")
            return
        
        req = SidescanSettings.Request()
        req.command = command
        req.side = self.ss_side_var.get()
        if command == "set":
            req.mode = self.ss_mode_var.get()
            req.method = self.ss_method_var.get()
            req.beams = self.ss_beams_var.get()
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.status_var.set("Sidescan settings updated")
            self.log(f"✓ SUCCESS: Sidescan {command}")
            self.log(f"  Response: {future.result().message}")
            
            # Parse and update GUI values for get command
            if command == "get":
                self._parse_sidescan_response(future.result().message, req.side)
        else:
            msg = future.result().message if future.result() else "No response"
            self.status_var.set(f"Failed: {msg}")
            self.log(f"✗ FAILED: {msg}")
    
    def _parse_sidescan_response(self, response, side):
        """Parse sidescan response and update GUI values"""
        import re
        if not response:
            return
        
        # Response format: "okay port(mode=incoherent method=rms beams=001100000) stbd(mode=coherent method=rms beams=000111110)"
        if side == "port" or side == "both":
            port_match = re.search(r'port\(([^)]+)\)', response)
            if port_match:
                self._parse_sidescan_side_values(port_match.group(1))
        elif side == "starboard" or side == "stbd":
            stbd_match = re.search(r'stbd\(([^)]+)\)', response)
            if stbd_match:
                self._parse_sidescan_side_values(stbd_match.group(1))
        
        # If both sides and GUI shows both, parse port
        if side == "both":
            port_match = re.search(r'port\(([^)]+)\)', response)
            if port_match:
                self._parse_sidescan_side_values(port_match.group(1))
        
        self.log("  GUI values updated from current settings")
    
    def _parse_sidescan_side_values(self, values_str):
        """Parse sidescan values from a side's response"""
        import re
        mode_match = re.search(r'mode=(\w+)', values_str)
        method_match = re.search(r'method=(\w+)', values_str)
        beams_match = re.search(r'beams=([01]+)', values_str)
        
        if mode_match:
            self.ss_mode_var.set(mode_match.group(1))
        if method_match:
            self.ss_method_var.set(method_match.group(1))
        if beams_match:
            self.ss_beams_var.set(beams_match.group(1))
    
    def sidescan3d_settings(self, command, side_override=None):
        """Call sidescan3d settings service"""
        self.status_var.set(f"Sidescan3D: {command}...")
        side = side_override if side_override else self.ss3d_side_var.get()
        self.log(f"Calling /sonar/sidescan3d with command={command}, side={side}")
        self.window.update()
        
        client = self.node.create_client(Sidescan3DSettings, '/sonar/sidescan3d')
        if not client.wait_for_service(timeout_sec=1.0):
            self.log("ERROR: sidescan3d service unavailable")
            return
        
        req = Sidescan3DSettings.Request()
        req.command = command
        req.side = side
        if command == "set":
            req.angles = int(self.ss3d_angles_var.get())
            req.smoothing = int(self.ss3d_smoothing_var.get())
            req.threshold = float(self.ss3d_threshold_var.get())
            req.tolerance = float(self.ss3d_tolerance_var.get())
            req.mindepth = float(self.ss3d_mindepth_var.get())
            req.maxdepth = float(self.ss3d_maxdepth_var.get())
            req.swath = float(self.ss3d_swath_var.get())
            req.amp = float(self.ss3d_amp_var.get())
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.status_var.set("Sidescan3D settings updated")
            self.log(f"✓ SUCCESS: Sidescan3D {command}")
            self.log(f"  Response: {future.result().message}")
            
            # Parse and update GUI values for get command
            if command == "get":
                self._parse_sidescan3d_response(future.result().message, side)
        else:
            msg = future.result().message if future.result() else "No response"
            self.status_var.set(f"Failed: {msg}")
            self.log(f"✗ FAILED: {msg}")
    
    def _parse_sidescan3d_response(self, response, side):
        """Parse sidescan3d response and update GUI values"""
        import re
        if not response:
            return
        
        # Response format: "okay port(angles=2 smoothing=20 threshold=-100 tolerance=0.1) stbd(angles=3 smoothing=20 threshold=-100 tolerance=0.1) (mindepth=1 maxdepth=200 swath=12 amp=0.3)"
        
        # Parse side-specific settings (port or stbd)
        if side == "port" or side == "both":
            port_match = re.search(r'port\(([^)]+)\)', response)
            if port_match:
                self._parse_sidescan3d_side_values(port_match.group(1))
        elif side == "stbd" or side == "starboard":
            stbd_match = re.search(r'stbd\(([^)]+)\)', response)
            if stbd_match:
                self._parse_sidescan3d_side_values(stbd_match.group(1))
        
        # Parse image filtering settings (applies to both sides)
        # Format: (mindepth=1 maxdepth=200 swath=12 amp=0.3)
        filter_match = re.search(r'\(mindepth=([\d.]+)\s+maxdepth=([\d.]+)\s+swath=([\d.]+)\s+amp=([\d.]+)\)', response)
        if filter_match:
            self.ss3d_mindepth_var.set(float(filter_match.group(1)))
            self.ss3d_maxdepth_var.set(float(filter_match.group(2)))
            self.ss3d_swath_var.set(float(filter_match.group(3)))
            self.ss3d_amp_var.set(float(filter_match.group(4)))
        
        self.log("  GUI values updated from current settings")
    
    def _parse_sidescan3d_side_values(self, values_str):
        """Parse sidescan3d values from a side's response"""
        import re
        angles_match = re.search(r'angles=(\d+)', values_str)
        smoothing_match = re.search(r'smoothing=(\d+)', values_str)
        threshold_match = re.search(r'threshold=([-\d.]+)', values_str)
        tolerance_match = re.search(r'tolerance=([\d.]+)', values_str)
        
        if angles_match:
            self.ss3d_angles_var.set(int(angles_match.group(1)))
        if smoothing_match:
            self.ss3d_smoothing_var.set(int(smoothing_match.group(1)))
        if threshold_match:
            self.ss3d_threshold_var.set(float(threshold_match.group(1)))
        if tolerance_match:
            self.ss3d_tolerance_var.set(float(tolerance_match.group(1)))
    
    def bathymetry_settings(self, command):
        """Call bathymetry settings service"""
        self.status_var.set(f"Bathymetry: {command}...")
        self.log(f"Calling /sonar/bathymetry with command={command}")
        self.window.update()
        
        client = self.node.create_client(BathymetrySettings, '/sonar/bathymetry')
        if not client.wait_for_service(timeout_sec=1.0):
            self.log("ERROR: bathymetry service unavailable")
            return
        
        req = BathymetrySettings.Request()
        req.command = command
        if command == "set":
            # Point rejection settings
            req.mindepth = float(self.bathy_mindepth_var.get())
            req.maxdepth = float(self.bathy_maxdepth_var.get())
            req.swath = float(self.bathy_swath_var.get())
            
            # Binning settings
            if self.bathy_binning_enable.get():
                req.binning_mode = self.bathy_binning_mode_var.get()
                req.binning_count = int(self.bathy_binning_count_var.get())
                req.binning_width = float(self.bathy_binning_width_var.get())
            else:
                req.binning_mode = ""
            
            # Bottom track settings
            if self.bathy_bt_enable.get():
                req.bottomtrack_mode = self.bathy_bt_mode_var.get()
                req.bottomtrack_cells = int(self.bathy_bt_cells_var.get())
                req.bottomtrack_width = float(self.bathy_bt_width_var.get())
                req.bottomtrack_height = float(self.bathy_bt_height_var.get())
                req.bottomtrack_heightp = int(self.bathy_bt_heightp_var.get())
                req.bottomtrack_alpha = float(self.bathy_bt_alpha_var.get())
            else:
                req.bottomtrack_mode = ""
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.status_var.set("Bathymetry settings updated")
            self.log(f"✓ SUCCESS: Bathymetry {command}")
            self.log(f"  Response: {future.result().message}")
            
            # Parse and update GUI values for get command
            if command == "get":
                self._parse_bathymetry_response(future.result().message)
        else:
            msg = future.result().message if future.result() else "No response"
            self.status_var.set(f"Failed: {msg}")
            self.log(f"✗ FAILED: {msg}")
    
    def _parse_bathymetry_response(self, response):
        """Parse bathymetry response and update GUI values"""
        import re
        if not response:
            return
        
        # Response format: "okay (mindepth=0.5 maxdepth=25 crosstrack=5 binning=equidistant:1440:0.2 bottomtrack=cartesian:200:1:0.5:10:0.5)"
        mindepth_match = re.search(r'mindepth=([-\d.]+)', response)
        maxdepth_match = re.search(r'maxdepth=([\d.]+)', response)
        swath_match = re.search(r'(?:swath|crosstrack)=([\d.]+)', response)
        
        if mindepth_match:
            self.bathy_mindepth_var.set(float(mindepth_match.group(1)))
        if maxdepth_match:
            self.bathy_maxdepth_var.set(float(maxdepth_match.group(1)))
        if swath_match:
            self.bathy_swath_var.set(float(swath_match.group(1)))
        
        # Parse binning: binning=equidistant:1440:0.2
        binning_match = re.search(r'binning=(\w+):([\d]+):([\d.]+)', response)
        if binning_match:
            self.bathy_binning_enable.set(True)
            self.bathy_binning_mode_var.set(binning_match.group(1))
            self.bathy_binning_count_var.set(int(binning_match.group(2)))
            self.bathy_binning_width_var.set(float(binning_match.group(3)))
        
        # Parse bottomtrack: bottomtrack=cartesian:200:1:0.5:10:0.5
        bt_match = re.search(r'bottomtrack=(\w+):([\d]+):([\d.]+):([\d.]+):([\d]+):([\d.]+)', response)
        if bt_match:
            self.bathy_bt_enable.set(True)
            self.bathy_bt_mode_var.set(bt_match.group(1))
            self.bathy_bt_cells_var.set(int(bt_match.group(2)))
            self.bathy_bt_width_var.set(float(bt_match.group(3)))
            self.bathy_bt_height_var.set(float(bt_match.group(4)))
            self.bathy_bt_heightp_var.set(int(bt_match.group(5)))
            self.bathy_bt_alpha_var.set(float(bt_match.group(6)))
        
        self.log("  GUI values updated from current settings")
    
    def commit_settings(self):
        """Commit changes to sonar"""
        self.status_var.set("Committing settings...")
        self.log("Calling /sonar/commit")
        self.window.update()
        
        client = self.node.create_client(CommitSettings, '/sonar/commit')
        if not client.wait_for_service(timeout_sec=1.0):
            self.log("ERROR: commit service unavailable")
            return
        
        req = CommitSettings.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.status_var.set("Settings committed successfully")
            self.log("✓ SUCCESS: Settings committed")
            self.log(f"  Response: {future.result().message}")
        else:
            msg = future.result().message if future.result() else "No response"
            self.status_var.set(f"Commit failed: {msg}")
            self.log(f"✗ FAILED: {msg}")
    
    def file_control(self, command):
        """Call file control service"""
        self.status_var.set(f"File: {command}...")
        self.log(f"Calling /sonar/file with command={command}")
        self.window.update()
        
        client = self.node.create_client(FileControl, '/sonar/file')
        if not client.wait_for_service(timeout_sec=1.0):
            self.log("ERROR: file control service unavailable")
            return
        
        req = FileControl.Request()
        req.command = command
        if command == "open":
            req.filename = self.file_path_var.get()
            if not req.filename:
                self.log("ERROR: No file specified")
                return
        elif command == "speed":
            req.speed = float(self.file_speed_var.get())
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            msg = f"File {command} successful"
            if command == "status" and future.result().file_name:
                msg += f" (file={future.result().file_name}, rate={future.result().ratehz:.1f}Hz)"
            self.status_var.set(msg)
            self.log(f"✓ SUCCESS: {msg}")
            self.log(f"  Response: {future.result().message}")
        else:
            msg = future.result().message if future.result() else "No response"
            self.status_var.set(f"Failed: {msg}")
            self.log(f"✗ FAILED: {msg}")
    
    def record_control(self, command):
        """Call record control service"""
        self.status_var.set(f"Record: {command}...")
        self.log(f"Calling /sonar/record with command={command}")
        self.window.update()
        
        client = self.node.create_client(RecordControl, '/sonar/record')
        if not client.wait_for_service(timeout_sec=1.0):
            self.log("ERROR: record control service unavailable")
            return
        
        req = RecordControl.Request()
        req.command = command
        if command == "start":
            req.filename = self.record_path_var.get()
            if not req.filename:
                self.log("ERROR: No file specified")
                return
            if not req.filename.endswith('.3dss-dx'):
                self.log("WARNING: Filename should have .3dss-dx extension")
            req.overwrite = self.record_overwrite_var.get()
            req.mode = self.record_mode_var.get()
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            msg = f"Record {command} successful"
            if command == "status" and future.result().file_name:
                msg += f" (file={future.result().file_name}, pings={future.result().pings})"
            self.status_var.set(msg)
            self.log(f"✓ SUCCESS: {msg}")
            self.log(f"  Response: {future.result().message}")
        else:
            msg = future.result().message if future.result() else "No response"
            self.status_var.set(f"Failed: {msg}")
            self.log(f"✗ FAILED: {msg}")
    
    def baud_settings(self, device):
        """Call baud settings service"""
        self.status_var.set(f"Baud: {device}...")
        self.log(f"Calling /sonar/baud for {device}")
        self.window.update()
        
        client = self.node.create_client(BaudSettings, '/sonar/baud')
        if not client.wait_for_service(timeout_sec=1.0):
            self.log("ERROR: baud service unavailable")
            return
        
        req = BaudSettings.Request()
        if device == "get":
            req.command = "get"
        else:
            req.command = "set"
            req.device = device
            if device == "gps":
                req.baudrate = int(self.gps_baud_var.get())
            else:  # mru
                req.baudrate = int(self.mru_baud_var.get())
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            msg = f"Baud {device} successful"
            if device == "get":
                msg += f" (GPS={future.result().gps_baud}, MRU={future.result().mru_baud})"
                self.gps_baud_var.set(str(future.result().gps_baud))
                self.mru_baud_var.set(str(future.result().mru_baud))
            self.status_var.set(msg)
            self.log(f"✓ SUCCESS: {msg}")
            self.log(f"  Response: {future.result().message}")
        else:
            msg = future.result().message if future.result() else "No response"
            self.status_var.set(f"Failed: {msg}")
            self.log(f"✗ FAILED: {msg}")
    
    def browse_file(self):
        """Browse for file to open"""
        filename = filedialog.askopenfilename(
            title="Select 3DSS-DX file",
            filetypes=[("3DSS-DX files", "*.3dss-dx"), ("All files", "*.*")]
        )
        if filename:
            self.file_path_var.set(filename)
    
    def browse_record_file(self):
        """Browse for file to record to"""
        filename = filedialog.asksaveasfilename(
            title="Select output file",
            defaultextension=".3dss-dx",
            filetypes=[("3DSS-DX files", "*.3dss-dx"), ("All files", "*.*")]
        )
        if filename:
            self.record_path_var.set(filename)
    
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
