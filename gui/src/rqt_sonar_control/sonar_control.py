#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QSlider, QFormLayout, QDoubleSpinBox
from rqt_gui_py.plugin import Plugin
from std_srvs.srv import Trigger
from pingdsp_msg.srv import SetSonarRange, SetSonarGain, SetSoundVelocity

class SonarControlWidget(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        layout = QVBoxLayout()
        self.setLayout(layout)

        # Range control
        range_layout = QFormLayout()
        self.range_spin = QDoubleSpinBox()
        self.range_spin.setRange(1.0, 300.0)
        self.range_spin.setValue(50.0)
        range_btn = QPushButton('Set Range')
        range_btn.clicked.connect(self.set_range)
        range_layout.addRow('Range (m):', self.range_spin)
        range_layout.addRow('', range_btn)
        layout.addLayout(range_layout)

        # Gain control
        gain_layout = QFormLayout()
        self.gain_spin = QDoubleSpinBox()
        self.gain_spin.setRange(0.0, 40.0)
        self.gain_spin.setValue(20.0)
        gain_btn = QPushButton('Set Gain')
        gain_btn.clicked.connect(self.set_gain)
        gain_layout.addRow('Gain (dB):', self.gain_spin)
        gain_layout.addRow('', gain_btn)
        layout.addLayout(gain_layout)

        # Sound velocity
        sv_layout = QFormLayout()
        self.sv_spin = QDoubleSpinBox()
        self.sv_spin.setRange(1300.0, 1700.0)
        self.sv_spin.setValue(1500.0)
        sv_btn = QPushButton('Set Sound Velocity')
        sv_btn.clicked.connect(self.set_sound_velocity)
        sv_layout.addRow('Sound Velocity (m/s):', self.sv_spin)
        sv_layout.addRow('', sv_btn)
        layout.addLayout(sv_layout)

        # Ping button
        ping_btn = QPushButton('Ping')
        ping_btn.clicked.connect(self.ping)
        layout.addWidget(ping_btn)

        self.status_label = QLabel('Status: Ready')
        layout.addWidget(self.status_label)

    def set_range(self):
        client = self.node.create_client(SetSonarRange, '/sonar/set_range')
        if not client.wait_for_service(timeout_sec=1.0):
            self.status_label.setText('Status: set_range service unavailable')
            return
        req = SetSonarRange.Request()
        req.range = self.range_spin.value()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() and future.result().success:
            self.status_label.setText('Status: Range set')
        else:
            self.status_label.setText('Status: Failed to set range')

    def set_gain(self):
        client = self.node.create_client(SetSonarGain, '/sonar/set_gain')
        if not client.wait_for_service(timeout_sec=1.0):
            self.status_label.setText('Status: set_gain service unavailable')
            return
        req = SetSonarGain.Request()
        req.gain = self.gain_spin.value()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() and future.result().success:
            self.status_label.setText('Status: Gain set')
        else:
            self.status_label.setText('Status: Failed to set gain')

    def set_sound_velocity(self):
        client = self.node.create_client(SetSoundVelocity, '/sonar/set_sound_velocity')
        if not client.wait_for_service(timeout_sec=1.0):
            self.status_label.setText('Status: set_sound_velocity service unavailable')
            return
        req = SetSoundVelocity.Request()
        req.sound_velocity = self.sv_spin.value()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() and future.result().success:
            self.status_label.setText('Status: Sound velocity set')
        else:
            self.status_label.setText('Status: Failed to set sound velocity')

    def ping(self):
        client = self.node.create_client(Trigger, '/sonar/ping')
        if not client.wait_for_service(timeout_sec=1.0):
            self.status_label.setText('Status: ping service unavailable')
            return
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() and future.result().success:
            self.status_label.setText('Status: Ping sent')
        else:
            self.status_label.setText('Status: Failed to ping')

class SonarControlPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        rclpy.init(args=None)
        self.node = rclpy.create_node('rqt_sonar_control')
        self._widget = SonarControlWidget(self.node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self.node.destroy_node()
        rclpy.shutdown()
