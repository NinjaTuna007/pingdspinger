#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox, simpledialog
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
from collections import deque
from datetime import datetime

import numpy as np

# Live sidescan viewing renders inside this GUI (no ROS image topic, so bags
# are not bloated). It needs PIL + the pure image maths from pingdsp_driver and
# the Ping3DSS message. If any are missing the control GUI still works; the
# Sidescan tab just shows why it is unavailable.
try:
    from PIL import Image as PILImage, ImageTk
    from pingdsp_driver import sidescan_image as ssi
    from pingdsp_msg.msg import Ping3DSS
    _HAVE_SIDESCAN = True
    _SS_IMPORT_ERROR = None
except Exception as _ss_err:  # pragma: no cover - depends on install env
    PILImage = ImageTk = ssi = Ping3DSS = None
    _HAVE_SIDESCAN = False
    _SS_IMPORT_ERROR = _ss_err


class SonarControlGUI:
    def __init__(self, node):
        self.node = node
        self.window = tk.Tk()
        self.window.title("3DSS-DX Sonar Control")
        self.window.geometry("1280x820")
        
        # Status label at top
        self.status_var = tk.StringVar(value="Ready")
        status_frame = ttk.Frame(self.window, padding=5)
        status_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(status_frame, textvariable=self.status_var, relief="sunken", anchor="w").pack(fill="x")
        
        # Main split: tabs on top, console log below, joined by a draggable
        # sash so the console vs tab-area sizes can be set by click-dragging.
        self.main_pane = ttk.PanedWindow(self.window, orient="vertical")
        self.main_pane.pack(fill="both", expand=True, padx=10, pady=5)

        # Create notebook for tabs (top pane)
        self.notebook = ttk.Notebook(self.main_pane)
        self.main_pane.add(self.notebook, weight=4)
        
        # Live sidescan state (buffer + renderer) must exist before its tab.
        self._init_sidescan_state()
        
        # Create tabs (Sidescan first so the live waterfall is front and centre)
        self.create_sidescan_tab()
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
        
        # Console log (bottom pane of the main split)
        console_frame = ttk.LabelFrame(self.main_pane, text="Console Log",
                                       padding=5)
        self.main_pane.add(console_frame, weight=1)
        
        self.console = scrolledtext.ScrolledText(console_frame, height=8, state='disabled', wrap='word')
        self.console.pack(fill="both", expand=True)
        # Colour-code the log; commands/success/errors are also bold.
        import tkinter.font as tkfont
        try:
            _bold = tkfont.Font(font=self.console.cget('font'))
            _bold.configure(weight='bold')
        except Exception:  # noqa: BLE001 - fall back to colour-only
            _bold = None
        self.console.tag_config('time', foreground='#888888')
        self.console.tag_config('command', foreground='#1565c0')   # blue
        self.console.tag_config('response', foreground='#00695c')  # teal
        self.console.tag_config('success', foreground='#2e7d32')   # green
        self.console.tag_config('error', foreground='#c62828')     # red
        self.console.tag_config('info', foreground='#333333')      # default
        if _bold is not None:
            for _t in ('command', 'success', 'error'):
                self.console.tag_config(_t, font=_bold)

        # Keep every line in memory so the category filters below can
        # show/hide whole classes of message without losing them.
        self.log_entries = []
        self.log_filters = {
            'command': tk.BooleanVar(value=True),
            'response': tk.BooleanVar(value=True),
            'success': tk.BooleanVar(value=True),
            'error': tk.BooleanVar(value=True),
            'info': tk.BooleanVar(value=True),
        }

        ctrl_row = ttk.Frame(console_frame)
        ctrl_row.pack(fill="x", pady=2)
        ttk.Button(ctrl_row, text="Clear Log",
                   command=self.clear_log).pack(side="left", padx=4)
        ttk.Label(ctrl_row, text="Show:").pack(side="left", padx=(10, 2))
        for _key, _lbl in (('command', 'Commands'), ('response', 'Responses'),
                           ('success', 'Success'), ('error', 'Errors'),
                           ('info', 'Info')):
            ttk.Checkbutton(ctrl_row, text=_lbl,
                            variable=self.log_filters[_key],
                            command=self._rebuild_console).pack(
                                side="left", padx=2)
        
        self.log("GUI initialized. Ready to control sonar.")
        
        # Subscribe to raw pings and start the live render loop.
        self._start_sidescan_stream()
    
    @staticmethod
    def _classify_log(message):
        """Pick a colour tag from the message prefix (commands/responses/etc)."""
        m = message.lstrip()
        if m.startswith("Calling "):
            return 'command'
        if m.startswith("\u2713"):          # checkmark -> success
            return 'success'
        if m.startswith("Response:"):
            return 'response'
        if (m.startswith("\u2717")           # cross mark -> failure
                or m.startswith("ERROR")
                or m.startswith("Failed")):
            return 'error'
        return 'info'

    # Direction markers: sent commands get an arrow out, received lines an
    # arrow in, status lines a tick/cross. Reads like a transcript.
    _LOG_MARKERS = {'command': '\u2192 ', 'response': '\u2190 ',
                    'success': '\u2713 ', 'error': '\u2717 ', 'info': '  '}

    def _insert_line(self, timestamp, body, tag):
        """Write one already-classified line to the console widget."""
        marker = self._LOG_MARKERS.get(tag, '  ')
        self.console.config(state='normal')
        self.console.insert('end', f"[{timestamp}] ", 'time')
        self.console.insert('end', f"{marker}{body}\n", tag)
        self.console.see('end')
        self.console.config(state='disabled')

    def log(self, message, level=None):
        """Add a timestamped, colour-coded message to the console log.

        ``level`` forces a tag ('command', 'response', 'success', 'error',
        'info'); when omitted it is inferred from the message prefix so all the
        existing call sites colour themselves automatically. Lines are stored so
        the category filters can hide/show them, and rendered as a transcript
        with direction arrows.
        """
        timestamp = datetime.now().strftime("%H:%M:%S")
        tag = level or self._classify_log(message)
        body = message.lstrip()
        if body[:1] in ('\u2713', '\u2717'):   # drop a caller's embedded glyph
            body = body[1:].lstrip()
        # Early-return errors (e.g. "ERROR: <svc> service unavailable") otherwise
        # leave the status bar stuck on a stale "...ing..." message. Surface them
        # there. (The post-call "FAILED:" branches set their own status first.)
        if (tag == 'error' and body.startswith('ERROR')
                and hasattr(self, 'status_var')):
            self.status_var.set(body)
        self.log_entries.append((timestamp, body, tag))
        if self.log_filters[tag].get():
            self._insert_line(timestamp, body, tag)
        self.node.get_logger().info(message)

    def _rebuild_console(self):
        """Re-render the console honouring the current category filters."""
        self.console.config(state='normal')
        self.console.delete('1.0', 'end')
        self.console.config(state='disabled')
        for timestamp, body, tag in self.log_entries:
            if self.log_filters[tag].get():
                self._insert_line(timestamp, body, tag)
    
    def clear_log(self):
        """Clear the console log and the stored history."""
        self.log_entries.clear()
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
    
    # ========================================================================
    # Live Sidescan viewer (renders in-GUI; no ROS image topic -> no bag bloat)
    # ========================================================================
    
    # Built-in, read-only sidescan preset. These are the shipped defaults; the
    # 'Default' entry in the preset picker always maps here and cannot be
    # overwritten or deleted. User presets live in gui/sidescan/config/.
    SS_DEFAULT_PRESET = {
        'num_pings': 2048,
        'width': 2048,
        'log_min': 11.5,
        'log_max': 15.0,
        'gamma': 1.0,
        'nadir_bins': 0,
        'flatten': 0.70,
        'clahe': 0.5,
        'despeckle': 0,
        'zoom': 1.0,
        'rate': 15.0,
        'colormap': 'bronze',
    }

    def _init_sidescan_state(self):
        """Set up the rolling ping buffer, renderer and tunable parameters."""
        # The waterfall is drawn into a scrollable canvas: it is scaled to fill
        # the available width (times Zoom) and may be taller than the viewport,
        # so the vertical scrollbar lets you scroll down into older pings.
        self._ss_after_id = None
        self._ss_photo = None  # keep a ref so Tk does not GC the image
        self.ss_canvas = None
        self.ss_img_id = None
        self.ss_text_id = None
        
        if not _HAVE_SIDESCAN:
            return
        
        # Tunables, all live via the sliders below. The standalone
        # sidescan_viewer_node YAML mirrors these defaults. Defaults come from
        # gui/sidescan/config/sidescan_default.json (from SS_DEFAULT_PRESET
        # on first run), so they are editable on disk.
        d = self.ss_default_preset = self._ss_load_default()
        self.ss_num_var = tk.IntVar(value=d['num_pings'])
        self.ss_width_var = tk.IntVar(value=d['width'])
        self.ss_logmin_var = tk.DoubleVar(value=d['log_min'])
        self.ss_logmax_var = tk.DoubleVar(value=d['log_max'])
        self.ss_gamma_var = tk.DoubleVar(value=d['gamma'])
        self.ss_nadir_var = tk.IntVar(value=d['nadir_bins'])
        self.ss_flatten_var = tk.DoubleVar(value=d['flatten'])
        self.ss_clahe_var = tk.DoubleVar(value=d['clahe'])
        self.ss_despeckle_var = tk.IntVar(value=d['despeckle'])
        self.ss_colormap_var = tk.StringVar(value=d['colormap'])
        self.ss_rate_var = tk.DoubleVar(value=d['rate'])
        self.ss_paused = tk.BooleanVar(value=False)
        # Display: zoom multiplier on top of fit-to-width, and whether to keep
        # the newest ping pinned at the top (off = free scroll through history).
        self.ss_zoom_var = tk.DoubleVar(value=d['zoom'])
        self.ss_follow = tk.BooleanVar(value=True)
        self.ss_clahe_grid = 8
        # Map preset keys <-> live Tk vars so presets can be saved/loaded.
        self.ss_param_vars = {
            'num_pings': self.ss_num_var, 'width': self.ss_width_var,
            'log_min': self.ss_logmin_var, 'log_max': self.ss_logmax_var,
            'gamma': self.ss_gamma_var, 'nadir_bins': self.ss_nadir_var,
            'flatten': self.ss_flatten_var, 'clahe': self.ss_clahe_var,
            'despeckle': self.ss_despeckle_var, 'zoom': self.ss_zoom_var,
            'rate': self.ss_rate_var, 'colormap': self.ss_colormap_var,
        }
        
        self.ss_num_pings = int(self.ss_num_var.get())
        self.ss_render_width = int(self.ss_width_var.get())
        self.ss_lock = threading.Lock()
        # Keep each ping's RAW across-track samples so both the ping count and
        # the per-row width can be resized live without discarding the image:
        # ss_rows holds the raw amplitudes; ss_log_rows the cached log1p row
        # binned to the *current* width (rebuilt only when the width changes).
        self.ss_rows = deque(maxlen=self.ss_num_pings)
        self.ss_log_rows = deque(maxlen=self.ss_num_pings)
        self.ss_ping_count = 0
    
    def create_sidescan_tab(self):
        """Create the live sidescan waterfall tab with all knobs as sliders."""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Sidescan")
        self.ss_tab = tab
        
        if not _HAVE_SIDESCAN:
            msg = ("Live sidescan viewing is unavailable.\n\n"
                   f"Import error: {_SS_IMPORT_ERROR}\n\n"
                   "Need Pillow + a built/sourced workspace so 'pingdsp_driver'"
                   " and 'pingdsp_msg' import.")
            ttk.Label(tab, text=msg, justify="left",
                      foreground="red").pack(anchor="w", padx=15, pady=15)
            return
        
        # Waterfall (left) and controls (right) joined by a draggable sash, so
        # the control sidebar width can be resized by click-dragging.
        body = ttk.PanedWindow(tab, orient="horizontal")
        body.pack(fill="both", expand=True)
        
        image_frame = ttk.LabelFrame(
            body, text="Waterfall (newest on top - scroll down for the past)",
            padding=5)
        body.add(image_frame, weight=5)
        # Scrollable canvas: the image fills the width and can exceed the
        # viewport height so you can scroll back through older pings.
        canvas = tk.Canvas(image_frame, background="#101010",
                           highlightthickness=0)
        vbar = ttk.Scrollbar(image_frame, orient="vertical",
                             command=canvas.yview)
        hbar = ttk.Scrollbar(image_frame, orient="horizontal",
                             command=canvas.xview)
        canvas.configure(yscrollcommand=vbar.set, xscrollcommand=hbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        vbar.grid(row=0, column=1, sticky="ns")
        hbar.grid(row=1, column=0, sticky="ew")
        image_frame.rowconfigure(0, weight=1)
        image_frame.columnconfigure(0, weight=1)
        self.ss_canvas = canvas
        self.ss_img_id = None
        self.ss_text_id = canvas.create_text(
            12, 12, anchor="nw", fill="#cccccc",
            text="Waiting for pings on sonar/ping...")
        # Scroll with the mouse wheel anywhere over the waterfall.
        canvas.bind("<MouseWheel>", self._ss_on_wheel)
        canvas.bind("<Button-4>", self._ss_on_wheel)
        canvas.bind("<Button-5>", self._ss_on_wheel)
        
        ctrl = ttk.LabelFrame(body, text="Visualisation", padding=8)
        body.add(ctrl, weight=1)
        
        def add_slider(label, var, lo, hi, res, cmd=None):
            row = ttk.Frame(ctrl)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=label, width=15).pack(side="left")
            tk.Scale(row, variable=var, from_=lo, to=hi, resolution=res,
                     orient="horizontal", length=210, command=cmd).pack(
                         side="left", fill="x", expand=True)
        
        # --- Presets (Default is built-in and read-only) ---
        prow = ttk.Frame(ctrl)
        prow.pack(fill="x", pady=(0, 2))
        ttk.Label(prow, text="Preset", width=15).pack(side="left")
        self.ss_preset_var = tk.StringVar(value="Default")
        self.ss_preset_combo = ttk.Combobox(
            prow, textvariable=self.ss_preset_var, state="readonly", width=14)
        self.ss_preset_combo.pack(side="left", fill="x", expand=True)
        self.ss_preset_combo.bind(
            "<<ComboboxSelected>>", lambda e: self.ss_preset_load())
        pbtn = ttk.Frame(ctrl)
        pbtn.pack(fill="x", pady=(0, 6))
        ttk.Button(pbtn, text="Save As...",
                   command=self.ss_preset_save).pack(side="left", padx=2)
        ttk.Button(pbtn, text="Delete",
                   command=self.ss_preset_delete).pack(side="left", padx=2)
        self._ss_refresh_preset_list()
        
        add_slider("Pings (rows)", self.ss_num_var, 64, 4096, 64,
                   self._ss_set_num)
        add_slider("Width (px)", self.ss_width_var, 128, 4096, 128,
                   self._ss_set_width)
        add_slider("Log min", self.ss_logmin_var, 6.0, 18.0, 0.1)
        add_slider("Log max", self.ss_logmax_var, 6.0, 20.0, 0.1)
        add_slider("Gamma", self.ss_gamma_var, 0.2, 3.0, 0.05)
        add_slider("Nadir bins", self.ss_nadir_var, 0, 512, 1)
        add_slider("Flatten", self.ss_flatten_var, 0.0, 1.0, 0.05)
        add_slider("CLAHE clip", self.ss_clahe_var, 0.0, 8.0, 0.5)
        add_slider("Despeckle", self.ss_despeckle_var, 0, 9, 1)
        add_slider("Zoom", self.ss_zoom_var, 0.25, 4.0, 0.05)
        add_slider("Refresh (Hz)", self.ss_rate_var, 1.0, 15.0, 1.0)
        
        cmap_row = ttk.Frame(ctrl)
        cmap_row.pack(fill="x", pady=4)
        ttk.Label(cmap_row, text="Colormap", width=15).pack(side="left")
        ttk.Combobox(cmap_row, textvariable=self.ss_colormap_var,
                     values=["copper", "bronze", "gray"], state="readonly",
                     width=12).pack(side="left")
        
        btns = ttk.Frame(ctrl)
        btns.pack(fill="x", pady=8)
        ttk.Button(btns, text="Reset", command=self.ss_reset).pack(
            side="left", padx=3)
        ttk.Button(btns, text="Save PNG", command=self.ss_save).pack(
            side="left", padx=3)
        ttk.Checkbutton(btns, text="Pause", variable=self.ss_paused).pack(
            side="left", padx=3)
        ttk.Checkbutton(btns, text="Follow newest",
                        variable=self.ss_follow).pack(side="left", padx=3)
        
        self.ss_info_var = tk.StringVar(value="no pings yet")
        ttk.Label(ctrl, textvariable=self.ss_info_var,
                  foreground="grey").pack(anchor="w", pady=(6, 0))
    
    def _start_sidescan_stream(self):
        """Subscribe to raw pings and kick off the periodic render loop."""
        if not _HAVE_SIDESCAN:
            return
        # Match the driver's ping publisher QoS (default reliable, depth 10).
        self.ss_sub = self.node.create_subscription(
            Ping3DSS, 'sonar/ping', self.ss_ping_callback, 10)
        self._ss_after_id = self.window.after(300, self.ss_refresh)
    
    @staticmethod
    def _ss_bin_raw(raw, width):
        """Bin a raw across-track ping to ``width`` and convert to log1p(amp)."""
        return np.log1p(np.abs(ssi.resample_row(raw, width))).astype(np.float32)

    def _ss_set_num(self, _=None):
        """Resize the ping buffers in place, keeping the most recent rows."""
        n = max(int(self.ss_num_var.get()), 1)
        with self.ss_lock:
            self.ss_num_pings = n
            self.ss_rows = deque(self.ss_rows, maxlen=n)
            self.ss_log_rows = deque(self.ss_log_rows, maxlen=n)
    
    def _ss_set_width(self, _=None):
        """Re-bin the kept pings to the new width live - the image is kept."""
        w = max(int(self.ss_width_var.get()), 1)
        with self.ss_lock:
            if w == self.ss_render_width:
                return
            self.ss_render_width = w
            self.ss_log_rows = deque(
                (self._ss_bin_raw(r, w) for r in self.ss_rows),
                maxlen=self.ss_num_pings)
    
    def ss_ping_callback(self, msg):
        """Store one ping's raw row + its binned log row (ROS thread)."""
        try:
            port = np.asarray(msg.port_sidescan_samples, dtype=np.float32)
            stbd = np.asarray(msg.starboard_sidescan_samples, dtype=np.float32)
            combined = ssi.combine_ping(port, stbd)
            if combined.size == 0:
                return
            # Scrub non-finite samples before binning so they cannot smear into
            # neighbouring bins via the cumulative-sum resample.
            combined = np.nan_to_num(combined, nan=0.0, posinf=0.0, neginf=0.0)
            with self.ss_lock:
                w = self.ss_render_width
                self.ss_rows.appendleft(combined)
                self.ss_log_rows.appendleft(self._ss_bin_raw(combined, w))
                self.ss_ping_count += 1
        except Exception:  # noqa: BLE001 - never let a bad ping kill the GUI
            pass
    
    def ss_render_bgr(self):
        """Build the current waterfall as a BGR uint8 image, or None.

        Always renders at full sample resolution: the colour map / transfer
        are applied to the whole stacked log image and the on-screen scaling
        happens once, in :meth:`_ss_show_on_canvas`. (We deliberately do *not*
        pre-downsample the float log image here - resampling a scrolling
        waterfall that contains NaN-padded dropped pings made the starboard
        side shimmer.)
        """
        with self.ss_lock:
            rows = list(self.ss_log_rows)
            num = self.ss_num_pings
        if not rows:
            return None
        log_img = ssi.stack_log_rows(rows, num)
        if log_img is None:
            return None
        # Exclude the near-nadir band from the across-track stats and black it
        # out, without having to clear the buffer when the slider moves.
        nb = int(self.ss_nadir_var.get())
        if nb > 0 and 2 * nb < log_img.shape[1]:
            c = log_img.shape[1] // 2
            log_img[:, c - nb:c + nb] = np.nan
        fs = float(self.ss_flatten_var.get())
        if fs > 0.0:
            log_img = ssi.flatten_across_track(log_img, fs)
        gray = ssi.log_to_gray(log_img, float(self.ss_logmin_var.get()),
                               float(self.ss_logmax_var.get()),
                               float(self.ss_gamma_var.get()))
        cc = float(self.ss_clahe_var.get())
        if cc > 0.0:
            gray = ssi.apply_clahe(gray, cc, self.ss_clahe_grid)
            gray = ssi.blank_nadir(gray, nb)
        ds = int(self.ss_despeckle_var.get())
        if ds >= 3:
            if ds % 2 == 0:
                ds += 1
            gray = ssi.despeckle_gray(gray, ds)
        return ssi.apply_colormap(gray, self.ss_colormap_var.get())
    
    def _ss_on_wheel(self, event):
        """Scroll the waterfall vertically with the mouse wheel."""
        if self.ss_canvas is None:
            return
        if getattr(event, 'num', None) == 4:        # X11 scroll up
            delta = -1
        elif getattr(event, 'num', None) == 5:      # X11 scroll down
            delta = 1
        else:                                        # Windows/macOS
            delta = -1 if event.delta > 0 else 1
        self.ss_canvas.yview_scroll(delta, "units")

    def _ss_show_on_canvas(self, bgr):
        """Scale the waterfall to fill the canvas width (x Zoom) and draw it.

        The image keeps its aspect ratio, so when it is taller than the
        viewport the vertical scrollbar exposes the older pings below.
        """
        view_w = max(int(self.ss_canvas.winfo_width()), 1)
        if view_w < 10:   # not laid out yet; try again next tick
            return
        h0, w0 = bgr.shape[:2]
        zoom = max(float(self.ss_zoom_var.get()), 0.05)
        scale = (view_w / float(w0)) * zoom
        disp_w = max(int(round(w0 * scale)), 1)
        disp_h = max(int(round(h0 * scale)), 1)
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        img = PILImage.fromarray(rgb).resize((disp_w, disp_h),
                                             PILImage.BILINEAR)
        self._ss_photo = ImageTk.PhotoImage(img)
        if self.ss_img_id is None:
            self.ss_img_id = self.ss_canvas.create_image(
                0, 0, anchor="nw", image=self._ss_photo)
        else:
            self.ss_canvas.itemconfigure(self.ss_img_id, image=self._ss_photo)
            self.ss_canvas.coords(self.ss_img_id, 0, 0)
        if self.ss_text_id is not None:
            self.ss_canvas.itemconfigure(self.ss_text_id, state="hidden")
        self.ss_canvas.configure(scrollregion=(0, 0, disp_w, disp_h))
        # Keep the newest ping (top of the image) in view unless the user has
        # turned off follow to browse the past.
        if self.ss_follow.get():
            self.ss_canvas.yview_moveto(0.0)

    def ss_refresh(self):
        """Render the waterfall into the tab; reschedules itself (main thread)."""
        try:
            visible = str(self.notebook.select()) == str(self.ss_tab)
            if visible and not self.ss_paused.get():
                bgr = self.ss_render_bgr()
                if bgr is not None:
                    self._ss_show_on_canvas(bgr)
                    with self.ss_lock:
                        filled = len(self.ss_rows)
                    self.ss_info_var.set(
                        f"pings={self.ss_ping_count}  buffer={filled}/"
                        f"{self.ss_num_pings}  render={bgr.shape[1]}x"
                        f"{bgr.shape[0]}")
        except Exception as e:  # noqa: BLE001 - keep the loop alive
            self.node.get_logger().warn(f"sidescan render error: {e}")
        finally:
            hz = max(float(self.ss_rate_var.get()), 0.5)
            self._ss_after_id = self.window.after(
                max(int(1000.0 / hz), 30), self.ss_refresh)
    
    def ss_reset(self):
        """Clear the buffer and rebuild the waterfall from scratch."""
        with self.ss_lock:
            self.ss_rows.clear()
            self.ss_log_rows.clear()
            self.ss_ping_count = 0
        self.log("Sidescan buffer reset.")
    
    def ss_save(self):
        """Save the full-resolution waterfall to a user-chosen file.

        Opens a Save-As dialog defaulting to ``gui/sidescan/saved`` with a
        timestamped name; the next save reopens wherever the last one landed.
        """
        bgr = self.ss_render_bgr()
        if bgr is None:
            self.log("Sidescan: nothing to save yet.")
            return
        import os
        import cv2
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = filedialog.asksaveasfilename(
            title="Save sidescan image",
            initialdir=getattr(self, '_ss_last_save_dir', None)
            or self._ss_saved_dir(),
            initialfile=f"sidescan_{stamp}.png",
            defaultextension=".png",
            filetypes=[("PNG image", "*.png"),
                       ("JPEG image", "*.jpg *.jpeg"),
                       ("All files", "*.*")])
        if not path:
            self.log("Sidescan save cancelled.")
            return
        out_dir = os.path.dirname(os.path.abspath(path))
        os.makedirs(out_dir, exist_ok=True)
        if cv2.imwrite(path, bgr):
            self._ss_last_save_dir = out_dir
            self.log(f"Saved sidescan image: {path}")
        else:
            self.log(f"ERROR: could not write sidescan image to {path}")

    # ----- Sidescan files (config + saved images under gui/sidescan/) --------
    def _ss_base_dir(self):
        """Return gui/sidescan/ (base for the config + saved-image folders)."""
        import os
        return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "sidescan")

    def _ss_saved_dir(self):
        """Return gui/sidescan/saved/, creating it if needed."""
        import os
        d = os.path.join(self._ss_base_dir(), "saved")
        os.makedirs(d, exist_ok=True)
        return d

    def _ss_config_dir(self):
        """Return gui/sidescan/config/, creating it if needed."""
        import os
        cfg = os.path.join(self._ss_base_dir(), "config")
        os.makedirs(cfg, exist_ok=True)
        return cfg

    def _ss_default_path(self):
        """Path of the editable Default preset file in gui/sidescan/config/."""
        import os
        return os.path.join(self._ss_config_dir(), "sidescan_default.json")

    def _ss_load_default(self):
        """Return the Default preset, materialising it in gui/sidescan/config/.

        Starts from the built-in SS_DEFAULT_PRESET; if the JSON exists its
        values override (defaults are editable on disk); otherwise the file is
        written so it appears in the config dir. Logs nothing (this runs before
        the console exists) - errors go to stderr.
        """
        import json
        import os
        import sys
        params = dict(self.SS_DEFAULT_PRESET)
        path = self._ss_default_path()
        if os.path.exists(path):
            try:
                with open(path) as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    for k in params:
                        if data.get(k) is not None:
                            params[k] = data[k]
            except Exception as e:  # noqa: BLE001
                print(f"[sidescan] could not read default preset: {e}",
                      file=sys.stderr)
        else:
            try:
                with open(path, "w") as f:
                    json.dump(params, f, indent=2, sort_keys=True)
            except Exception as e:  # noqa: BLE001
                print(f"[sidescan] could not write default preset: {e}",
                      file=sys.stderr)
        return params

    def _ss_preset_path(self):
        """Return the JSON path for user presets, creating gui/sidescan/config/."""
        import os
        return os.path.join(self._ss_config_dir(), "sidescan_presets.json")

    def _ss_load_presets_file(self):
        """Load the user preset dict from disk (empty on missing/bad file)."""
        import json
        import os
        path = self._ss_preset_path()
        if not os.path.exists(path):
            return {}
        try:
            with open(path, "r") as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except Exception as e:  # noqa: BLE001
            self.log(f"ERROR: could not read sidescan presets: {e}")
            return {}

    def _ss_save_presets_file(self, presets):
        """Persist the user preset dict to disk."""
        import json
        try:
            with open(self._ss_preset_path(), "w") as f:
                json.dump(presets, f, indent=2, sort_keys=True)
        except Exception as e:  # noqa: BLE001
            self.log(f"ERROR: could not save sidescan presets: {e}")

    def _ss_refresh_preset_list(self):
        """Repopulate the preset combobox: Default first, then user presets."""
        names = ["Default"] + sorted(self._ss_load_presets_file().keys())
        self.ss_preset_combo.configure(values=names)

    def _ss_collect_params(self):
        """Snapshot the current slider/colormap values as a preset dict."""
        return {k: v.get() for k, v in self.ss_param_vars.items()}

    def _ss_apply_params(self, params):
        """Apply a preset dict to the live controls (and side effects)."""
        old_w = int(self.ss_width_var.get())
        for k, var in self.ss_param_vars.items():
            if params.get(k) is not None:
                try:
                    var.set(params[k])
                except Exception:  # noqa: BLE001 - skip incompatible values
                    pass
        # Resize the ping buffer, then re-bin to the new width if it changed
        # (both keep the accumulated image; width only rebins, never clears).
        self._ss_set_num()
        if int(self.ss_width_var.get()) != old_w:
            self._ss_set_width()

    def ss_preset_load(self):
        """Load the preset currently selected in the combobox."""
        name = self.ss_preset_var.get()
        if name == "Default":
            # Re-read from disk so on-the-fly edits to the file take effect.
            params = self._ss_load_default()
        else:
            params = self._ss_load_presets_file().get(name)
            if params is None:
                self.log(f"ERROR: sidescan preset '{name}' not found")
                return
        self._ss_apply_params(params)
        self.log(f"Loaded sidescan preset '{name}'", level='info')

    def ss_preset_save(self):
        """Save the current settings as a named user preset."""
        name = simpledialog.askstring(
            "Save preset", "Preset name:", parent=self.window)
        if not name:
            return
        name = name.strip()
        if name.lower() == "default":
            messagebox.showerror(
                "Save preset",
                "'Default' is reserved and cannot be overwritten.")
            return
        presets = self._ss_load_presets_file()
        presets[name] = self._ss_collect_params()
        self._ss_save_presets_file(presets)
        self._ss_refresh_preset_list()
        self.ss_preset_var.set(name)
        self.log(f"Saved sidescan preset '{name}'", level='info')

    def ss_preset_delete(self):
        """Delete the selected user preset (the Default is protected)."""
        name = self.ss_preset_var.get()
        if name == "Default":
            messagebox.showinfo(
                "Delete preset", "The Default preset cannot be deleted.")
            return
        presets = self._ss_load_presets_file()
        if name in presets:
            del presets[name]
            self._ss_save_presets_file(presets)
        self._ss_refresh_preset_list()
        self.ss_preset_var.set("Default")
        self.log(f"Deleted sidescan preset '{name}'", level='info')

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
