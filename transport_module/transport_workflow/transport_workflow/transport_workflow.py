#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Transport Workflow GUI

GUI for transport logic control with:
- Motor status display (ZD motor, Servo motor, Motor work)
- Microswitch status display
- State machine status display
- Manual control mode (disable state machine)
- Direct motor speed control
- State machine state switching
"""

import sys
import os
import warnings

warnings.filterwarnings('ignore', message='.*NumPy 1.x.*')
warnings.filterwarnings('ignore', message='.*_ARRAY_API.*')

# Fix Qt plugin conflict
_cv2_qt_path = os.path.expanduser('~/.local/lib/python3.10/site-packages/cv2/qt/plugins')
if os.path.exists(_cv2_qt_path):
    os.environ['QT_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins'

for var in ['QT_QPA_PLATFORM_PLUGIN_PATH', 'CV2_QT_PLUGIN_PATH']:
    if var in os.environ:
        del os.environ[var]

if 'QT_QPA_PLATFORM' not in os.environ:
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

import threading
from typing import Optional

import numpy as np
from collections import deque
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QGroupBox, QFrame, QComboBox,
    QDoubleSpinBox, QGridLayout, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64, Bool, String


# ============================================================================
# ROS Signal Bridge for Thread-Safe Qt Updates
# ============================================================================
class RosSignalBridge(QObject):
    """Bridge to safely emit Qt signals from ROS callbacks"""
    state_machine_state_updated = pyqtSignal(str)
    microswitch_status_updated = pyqtSignal(str)
    vertical_max_updated = pyqtSignal(str)
    vertical_min_updated = pyqtSignal(str)
    horizontal_max_updated = pyqtSignal(str)
    horizontal_min_updated = pyqtSignal(str)
    zd_motor_state_updated = pyqtSignal(str)
    servo_motor_state_updated = pyqtSignal(str)
    motor_work_state_updated = pyqtSignal(str)
    distance_sensor_updated = pyqtSignal(float)


# ============================================================================
# ROS2 Node
# ============================================================================
class TransportWorkflowNode(Node):
    """ROS2 Node for transport workflow GUI"""
    
    def __init__(self, signal_bridge: RosSignalBridge):
        super().__init__('transport_workflow')
        self.signal_bridge = signal_bridge
        
        self._setup_subscribers()
        self._setup_publishers()
        
        self.get_logger().info('Transport Workflow GUI Node initialized')
    
    def _setup_subscribers(self):
        """Setup subscribers for status updates"""
        self.state_machine_state_sub = self.create_subscription(
            String, 'transport_logic/state_machine_state',
            self._state_machine_state_callback, 10)
        self.microswitch_status_sub = self.create_subscription(
            String, 'transport_logic/microswitch_status',
            self._microswitch_status_callback, 10)
        self.vertical_max_sub = self.create_subscription(
            String, 'transport_logic/vertical_max',
            self._vertical_max_callback, 10)
        self.vertical_min_sub = self.create_subscription(
            String, 'transport_logic/vertical_min',
            self._vertical_min_callback, 10)
        self.horizontal_max_sub = self.create_subscription(
            String, 'transport_logic/horizontal_max',
            self._horizontal_max_callback, 10)
        self.horizontal_min_sub = self.create_subscription(
            String, 'transport_logic/horizontal_min',
            self._horizontal_min_callback, 10)
        self.zd_motor_state_sub = self.create_subscription(
            String, 'transport_logic/zd_motor_state',
            self._zd_motor_state_callback, 10)
        self.servo_motor_state_sub = self.create_subscription(
            String, 'transport_logic/servo_motor_state',
            self._servo_motor_state_callback, 10)
        self.motor_work_state_sub = self.create_subscription(
            String, 'transport_logic/motor_work_state',
            self._motor_work_state_callback, 10)
        self.distance_sensor_sub = self.create_subscription(
            Float64, 'distance/value',
            self._distance_sensor_callback, 10)
    
    def _setup_publishers(self):
        """Setup publishers for manual control"""
        self.manual_mode_pub = self.create_publisher(
            Bool, 'transport_logic/manual_mode', 10)
        self.set_state_pub = self.create_publisher(
            String, 'transport_logic/set_state', 10)
        self.confirm_wait_pub = self.create_publisher(
            Bool, 'transport_logic/confirm_wait', 10)
        self.zd_motor_speed_pub = self.create_publisher(
            Float64, 'transport_logic/zd_motor/manual_speed', 10)
        self.zd_motor_direction_pub = self.create_publisher(
            String, 'transport_logic/zd_motor/manual_direction', 10)
        self.servo_motor_speed_pub = self.create_publisher(
            Float64, 'transport_logic/servo_motor/manual_speed', 10)
        self.servo_motor_direction_pub = self.create_publisher(
            String, 'transport_logic/servo_motor/manual_direction', 10)
        self.servo_motor_enable_pub = self.create_publisher(
            Bool, 'transport_logic/servo_motor/manual_enable', 10)
    
    def _state_machine_state_callback(self, msg: String):
        self.signal_bridge.state_machine_state_updated.emit(msg.data)
    
    def _microswitch_status_callback(self, msg: String):
        self.signal_bridge.microswitch_status_updated.emit(msg.data)
    
    def _vertical_max_callback(self, msg: String):
        self.signal_bridge.vertical_max_updated.emit(msg.data)
    
    def _vertical_min_callback(self, msg: String):
        self.signal_bridge.vertical_min_updated.emit(msg.data)
    
    def _horizontal_max_callback(self, msg: String):
        self.signal_bridge.horizontal_max_updated.emit(msg.data)
    
    def _horizontal_min_callback(self, msg: String):
        self.signal_bridge.horizontal_min_updated.emit(msg.data)
    
    def _zd_motor_state_callback(self, msg: String):
        self.signal_bridge.zd_motor_state_updated.emit(msg.data)
    
    def _servo_motor_state_callback(self, msg: String):
        self.signal_bridge.servo_motor_state_updated.emit(msg.data)
    
    def _motor_work_state_callback(self, msg: String):
        self.signal_bridge.motor_work_state_updated.emit(msg.data)
    
    def _distance_sensor_callback(self, msg: Float64):
        # Only emit valid (non-NaN) values
        if not np.isnan(msg.data):
            self.signal_bridge.distance_sensor_updated.emit(msg.data)
    
    def publish_manual_mode(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.manual_mode_pub.publish(msg)
    
    def publish_set_state(self, state: str):
        msg = String()
        msg.data = state
        self.set_state_pub.publish(msg)
    
    def publish_confirm_wait(self):
        msg = Bool()
        msg.data = True
        self.confirm_wait_pub.publish(msg)
    
    def publish_zd_motor_speed(self, speed: float):
        msg = Float64()
        msg.data = float(speed)
        self.zd_motor_speed_pub.publish(msg)
    
    def publish_zd_motor_direction(self, direction: str):
        msg = String()
        msg.data = direction
        self.zd_motor_direction_pub.publish(msg)
    
    def publish_servo_motor_speed(self, speed: float):
        msg = Float64()
        msg.data = float(speed)
        self.servo_motor_speed_pub.publish(msg)
    
    def publish_servo_motor_direction(self, direction: str):
        msg = String()
        msg.data = direction
        self.servo_motor_direction_pub.publish(msg)
    
    def publish_servo_motor_enable(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.servo_motor_enable_pub.publish(msg)


# ============================================================================
# Motor Control Panel Widget
# ============================================================================
class MotorControlPanel(QGroupBox):
    """Control panel for a motor"""
    
    def __init__(self, motor_name: str, is_servo: bool = False, parent=None):
        super().__init__(motor_name, parent)
        self.motor_name = motor_name
        self.is_servo = is_servo
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout()
        layout.setSpacing(8)
        
        # Status display with gray background
        status_frame = QFrame()
        status_frame.setStyleSheet("""
            QFrame {
                background-color: #e0e0e0;
                border: 1px solid #c0c0c0;
                border-radius: 4px;
                padding: 8px;
            }
        """)
        status_layout = QGridLayout(status_frame)
        
        self.speed_label = QLabel("Speed: 0")
        status_layout.addWidget(self.speed_label, 0, 0)
        
        self.direction_label = QLabel("Direction: STOP")
        status_layout.addWidget(self.direction_label, 0, 1)
        
        # Note: Enable display removed for alignment - enable control still available via button
        
        layout.addWidget(status_frame)
        
        # Separator
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setStyleSheet("background-color: #c0c0c0;")
        layout.addWidget(line)
        
        # Speed control
        speed_label = QLabel("Speed Control:")
        speed_label.setFont(QFont("Arial", 9, QFont.Bold))
        layout.addWidget(speed_label)
        
        self.speed_slider = QSlider(Qt.Horizontal)
        if self.is_servo:
            self.speed_slider.setMinimum(0)
            self.speed_slider.setMaximum(2000)
        else:
            self.speed_slider.setMinimum(-5000)
            self.speed_slider.setMaximum(5000)
        self.speed_slider.setValue(0)
        layout.addWidget(self.speed_slider)
        
        input_layout = QHBoxLayout()
        self.speed_spinbox = QDoubleSpinBox()
        if self.is_servo:
            self.speed_spinbox.setMinimum(0)
            self.speed_spinbox.setMaximum(2000)
        else:
            self.speed_spinbox.setMinimum(-5000)
            self.speed_spinbox.setMaximum(5000)
        self.speed_spinbox.setValue(0)
        self.speed_spinbox.setSuffix(" RPM")
        self.speed_spinbox.setDecimals(1)
        self.speed_spinbox.setSingleStep(10)
        
        self.set_speed_btn = QPushButton("Set Speed")
        self.set_speed_btn.setStyleSheet("""
            QPushButton {
                background-color: #808080;
                color: #ffffff;
                border: 1px solid #606060;
                padding: 6px 12px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #909090; }
        """)
        
        input_layout.addWidget(self.speed_spinbox)
        input_layout.addWidget(self.set_speed_btn)
        layout.addLayout(input_layout)
        
        # Direction control
        direction_layout = QHBoxLayout()
        direction_label = QLabel("Direction:")
        direction_layout.addWidget(direction_label)
        
        self.direction_combo = QComboBox()
        self.direction_combo.addItems(["STOP", "FORWARD", "REVERSE"])
        direction_layout.addWidget(self.direction_combo)
        
        self.set_direction_btn = QPushButton("Set Direction")
        self.set_direction_btn.setStyleSheet("""
            QPushButton {
                background-color: #808080;
                color: #ffffff;
                border: 1px solid #606060;
                padding: 6px 12px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #909090; }
        """)
        direction_layout.addWidget(self.set_direction_btn)
        layout.addLayout(direction_layout)
        
        # Enable control removed for alignment (all three motors now have same layout)
        # Note: Enable functionality still exists but UI is hidden
        if self.is_servo:
            # Create enable button but keep it hidden - functionality still works via code
            self.enable_btn = QPushButton("Enable")
            self.enable_btn.setCheckable(True)
            self.enable_btn.setVisible(False)  # Hidden for alignment
            self.enable_btn.setStyleSheet("""
                QPushButton {
                    background-color: #606060;
                    color: #ffffff;
                    border: 1px solid #404040;
                    padding: 8px 16px;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:checked {
                    background-color: #008000;
                }
                QPushButton:hover { background-color: #707070; }
            """)
        else:
            # For non-servo motors, create a dummy enable_btn to avoid errors
            self.enable_btn = None
        
        self.setLayout(layout)
        
        # Connect internal signals
        self.speed_slider.valueChanged.connect(self._on_slider_changed)
        self.speed_spinbox.valueChanged.connect(self._on_spinbox_changed)
    
    def _on_slider_changed(self, value):
        self.speed_spinbox.blockSignals(True)
        self.speed_spinbox.setValue(value)
        self.speed_spinbox.blockSignals(False)
    
    def _on_spinbox_changed(self, value):
        self.speed_slider.blockSignals(True)
        self.speed_slider.setValue(int(value))
        self.speed_slider.blockSignals(False)
    
    def update_status(self, speed: float, direction: str, enabled: bool = None):
        """Update motor status display"""
        if self.is_servo:
            self.speed_label.setText(f"Speed: {speed:.1f} RPM")
        else:
            self.speed_label.setText(f"Speed: {int(speed)} RPM")
        self.direction_label.setText(f"Direction: {direction}")
        if self.is_servo and enabled is not None:
            # Enable label removed for alignment, but button still updates
            self.enable_btn.setChecked(enabled)
    
    def get_speed(self) -> float:
        return self.speed_spinbox.value()
    
    def get_direction(self) -> str:
        return self.direction_combo.currentText()
    
    def is_enabled(self) -> bool:
        if self.is_servo and self.enable_btn is not None:
            return self.enable_btn.isChecked()
        return False


# ============================================================================
# Distance Sensor Plot Widget
# ============================================================================
class DistanceSensorPlot(QGroupBox):
    """Distance sensor plot widget with real-time curve"""
    
    def __init__(self, parent=None):
        super().__init__("Distance Sensor", parent)
        self.max_points = 500  # Keep last 500 points
        self.time_data = deque(maxlen=self.max_points)
        self.distance_data = deque(maxlen=self.max_points)
        self.start_time = datetime.now()
        
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Create matplotlib figure
        self.figure = Figure(figsize=(8, 4), facecolor='white')
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlabel('Time (s)', fontsize=10)
        self.ax.set_ylabel('Distance (m)', fontsize=10)
        self.ax.set_title('Distance Sensor Real-time Plot', fontsize=12, fontweight='bold')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_ylim(0, 2.0)  # Set reasonable range for distance
        
        # Initialize empty line
        self.line, = self.ax.plot([], [], 'b-', linewidth=1.5, label='Distance')
        self.ax.axhline(y=0.8, color='r', linestyle='--', linewidth=1, label='Threshold (0.8m)')
        self.ax.legend(loc='upper right', fontsize=9)
        
        self.figure.tight_layout()
        layout.addWidget(self.canvas)
        
        # Current value display
        self.value_label = QLabel("Current: -- m")
        self.value_label.setStyleSheet("""
            QLabel {
                background-color: #f0f0f0;
                color: #000000;
                padding: 4px 12px;
                border: 1px solid #a0a0a0;
                border-radius: 4px;
                font-size: 12px;
                font-weight: bold;
            }
        """)
        self.value_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.value_label)
        
        self.setLayout(layout)
    
    def update_distance(self, distance: float):
        """Update plot with new distance value"""
        current_time = datetime.now()
        elapsed = (current_time - self.start_time).total_seconds()
        
        self.time_data.append(elapsed)
        self.distance_data.append(distance)
        
        # Update plot
        if len(self.time_data) > 0:
            self.line.set_data(list(self.time_data), list(self.distance_data))
            self.ax.set_xlim(max(0, elapsed - 30), max(30, elapsed))  # Show last 30 seconds
            
            # Auto-adjust y-axis if needed
            if len(self.distance_data) > 0:
                max_dist = max(self.distance_data)
                min_dist = min(self.distance_data)
                margin = (max_dist - min_dist) * 0.1 if max_dist > min_dist else 0.2
                self.ax.set_ylim(max(0, min_dist - margin), max_dist + margin)
        
        self.canvas.draw()
        
        # Update label
        self.value_label.setText(f"Current: {distance:.3f} m")


# ============================================================================
# Main Transport Workflow GUI Window
# ============================================================================
class TransportWorkflowGUI(QMainWindow):
    """Main transport workflow GUI window"""
    
    def __init__(self, ros_node: TransportWorkflowNode, signal_bridge: RosSignalBridge):
        super().__init__()
        self.ros_node = ros_node
        self.signal_bridge = signal_bridge
        
        self._manual_mode = True  # Default to manual mode
        self._setup_ui()
        self._connect_signals()
    
    def _setup_ui(self):
        self.setWindowTitle("Transport Workflow Control")
        self.setMinimumSize(1200, 800)
        # Allow window to resize properly
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Apply light gray/white theme
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
            }
            QGroupBox {
                background-color: #ffffff;
                border: 1px solid #c0c0c0;
                border-radius: 5px;
                margin-top: 12px;
                padding: 10px;
                font-size: 12px;
                font-weight: bold;
                color: #303030;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 15px;
                padding: 0 8px;
            }
            QLabel {
                color: #303030;
            }
            QSlider::groove:horizontal {
                border: 1px solid #a0a0a0;
                height: 6px;
                background: #d0d0d0;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #606060;
                border: 1px solid #404040;
                width: 16px;
                margin: -5px 0;
                border-radius: 8px;
            }
            QDoubleSpinBox, QSpinBox {
                background-color: #ffffff;
                color: #303030;
                border: 1px solid #a0a0a0;
                border-radius: 4px;
                padding: 4px;
            }
            QComboBox {
                background-color: #ffffff;
                color: #303030;
                border: 1px solid #a0a0a0;
                border-radius: 4px;
                padding: 4px;
            }
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # Add top stretch to push content down
        main_layout.addStretch(1)
        
        # Title
        title = QLabel("Transport Workflow Control System")
        title.setStyleSheet("""
            QLabel {
                font-size: 20px;
                font-weight: bold;
                color: #303030;
                padding: 5px;
            }
        """)
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        # Status display section (compact one-line layout)
        status_group = QGroupBox("Status Display")
        status_layout = QHBoxLayout(status_group)
        status_layout.setSpacing(10)
        status_layout.setContentsMargins(10, 10, 10, 10)
        
        # State machine status (compact)
        status_layout.addWidget(QLabel("State:"))
        self.state_machine_label = QLabel("INIT")
        self.state_machine_label.setStyleSheet("""
            QLabel {
                background-color: #f0f0f0;
                color: #000000;
                padding: 4px 12px;
                border: 1px solid #a0a0a0;
                border-radius: 4px;
                font-size: 12px;
                font-weight: bold;
                min-width: 60px;
            }
        """)
        self.state_machine_label.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.state_machine_label)
        
        # State machine control
        status_layout.addWidget(QLabel("Set:"))
        self.state_combo = QComboBox()
        self.state_combo.addItems(["INIT", "WAIT", "PRE", "WORK", "EMPTY", "FINISH"])
        self.state_combo.setMaximumWidth(100)
        status_layout.addWidget(self.state_combo)
        
        self.set_state_btn = QPushButton("Set")
        self.set_state_btn.setStyleSheet("""
            QPushButton {
                background-color: #808080;
                color: #ffffff;
                border: 1px solid #606060;
                padding: 4px 12px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #909090; }
        """)
        status_layout.addWidget(self.set_state_btn)
        
        # Confirm WAIT button (only visible when in WAIT state)
        self.confirm_wait_btn = QPushButton("Confirm to PRE")
        self.confirm_wait_btn.setStyleSheet("""
            QPushButton {
                background-color: #0066CC;
                color: #ffffff;
                border: 1px solid #0055AA;
                padding: 4px 12px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #0077DD; }
        """)
        self.confirm_wait_btn.setVisible(False)  # Hidden by default
        status_layout.addWidget(self.confirm_wait_btn)
        
        status_layout.addWidget(QFrame())  # Spacer
        
        # Manual mode toggle
        # Note: checked = manual mode (red), unchecked = auto mode (green)
        self.manual_mode_btn = QPushButton("Manual Mode: ON")
        self.manual_mode_btn.setCheckable(True)
        self.manual_mode_btn.setChecked(True)  # Default to manual mode
        self.manual_mode_btn.setStyleSheet("""
            QPushButton {
                background-color: #CC0000;
                color: #ffffff;
                border: 1px solid #AA0000;
                padding: 4px 12px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:checked {
                background-color: #CC0000;
            }
            QPushButton:!checked {
                background-color: #00AA00;
            }
            QPushButton:hover:checked { background-color: #DD0000; }
            QPushButton:hover:!checked { background-color: #00BB00; }
        """)
        status_layout.addWidget(self.manual_mode_btn)
        
        main_layout.addWidget(status_group)
        
        # Microswitch status section
        microswitch_group = QGroupBox("Microswitch Status")
        microswitch_layout = QHBoxLayout(microswitch_group)
        microswitch_layout.setSpacing(15)
        microswitch_layout.setContentsMargins(10, 10, 10, 10)
        
        # Vertical Max
        microswitch_layout.addWidget(QLabel("Vertical Max:"))
        self.vertical_max_label = QLabel("unknown")
        self.vertical_max_label.setStyleSheet("""
            QLabel {
                background-color: #f0f0f0;
                color: #000000;
                padding: 4px 12px;
                border: 1px solid #a0a0a0;
                border-radius: 4px;
                font-size: 12px;
                font-weight: bold;
                min-width: 100px;
            }
        """)
        self.vertical_max_label.setAlignment(Qt.AlignCenter)
        microswitch_layout.addWidget(self.vertical_max_label)
        
        # Vertical Min
        microswitch_layout.addWidget(QLabel("Vertical Min:"))
        self.vertical_min_label = QLabel("unknown")
        self.vertical_min_label.setStyleSheet("""
            QLabel {
                background-color: #f0f0f0;
                color: #000000;
                padding: 4px 12px;
                border: 1px solid #a0a0a0;
                border-radius: 4px;
                font-size: 12px;
                font-weight: bold;
                min-width: 100px;
            }
        """)
        self.vertical_min_label.setAlignment(Qt.AlignCenter)
        microswitch_layout.addWidget(self.vertical_min_label)
        
        # Horizontal Max
        microswitch_layout.addWidget(QLabel("Horizontal Max:"))
        self.horizontal_max_label = QLabel("unknown")
        self.horizontal_max_label.setStyleSheet("""
            QLabel {
                background-color: #f0f0f0;
                color: #000000;
                padding: 4px 12px;
                border: 1px solid #a0a0a0;
                border-radius: 4px;
                font-size: 12px;
                font-weight: bold;
                min-width: 100px;
            }
        """)
        self.horizontal_max_label.setAlignment(Qt.AlignCenter)
        microswitch_layout.addWidget(self.horizontal_max_label)
        
        # Horizontal Min
        microswitch_layout.addWidget(QLabel("Horizontal Min:"))
        self.horizontal_min_label = QLabel("unknown")
        self.horizontal_min_label.setStyleSheet("""
            QLabel {
                background-color: #f0f0f0;
                color: #000000;
                padding: 4px 12px;
                border: 1px solid #a0a0a0;
                border-radius: 4px;
                font-size: 12px;
                font-weight: bold;
                min-width: 100px;
            }
        """)
        self.horizontal_min_label.setAlignment(Qt.AlignCenter)
        microswitch_layout.addWidget(self.horizontal_min_label)
        
        microswitch_layout.addStretch()
        main_layout.addWidget(microswitch_group)
        
        # Motor control section (reordered: Horizontal, Vertical, Transport)
        motor_group = QGroupBox("Motor Control")
        motor_layout = QHBoxLayout(motor_group)
        motor_layout.setSpacing(10)
        
        self.horizontal_motor_panel = MotorControlPanel("Horizontal Motor", is_servo=True)
        self.vertical_motor_panel = MotorControlPanel("Vertical Motor", is_servo=False)
        self.transport_motor_panel = MotorControlPanel("Transport Motor", is_servo=False)
        
        # Set size policies - allow horizontal expansion but keep preferred vertical size
        self.horizontal_motor_panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.vertical_motor_panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.transport_motor_panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        
        # Set maximum heights to prevent excessive stretching
        self.horizontal_motor_panel.setMaximumHeight(500)
        self.vertical_motor_panel.setMaximumHeight(500)
        self.transport_motor_panel.setMaximumHeight(500)
        
        motor_layout.addWidget(self.horizontal_motor_panel, 1)
        motor_layout.addWidget(self.vertical_motor_panel, 1)
        motor_layout.addWidget(self.transport_motor_panel, 1)
        
        main_layout.addWidget(motor_group)  # No stretch factor - keep natural size
        
        # Distance sensor plot section
        self.distance_plot = DistanceSensorPlot()
        self.distance_plot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.distance_plot.setMinimumHeight(300)
        main_layout.addWidget(self.distance_plot)
        
        # Add bottom stretch to push content up
        main_layout.addStretch(1)
        
        # Status bar
        status_layout = QHBoxLayout()
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("background-color: #f0f0f0; color: #606060; font-size: 10px; padding: 3px;")
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        
        main_layout.addLayout(status_layout)
        
        central_widget.setLayout(main_layout)
    
    def _connect_signals(self):
        """Connect all signals"""
        # ROS signal bridge connections
        self.signal_bridge.state_machine_state_updated.connect(self._on_state_machine_state_updated)
        self.signal_bridge.microswitch_status_updated.connect(self._on_microswitch_status_updated)
        self.signal_bridge.vertical_max_updated.connect(self._on_vertical_max_updated)
        self.signal_bridge.vertical_min_updated.connect(self._on_vertical_min_updated)
        self.signal_bridge.horizontal_max_updated.connect(self._on_horizontal_max_updated)
        self.signal_bridge.horizontal_min_updated.connect(self._on_horizontal_min_updated)
        self.signal_bridge.zd_motor_state_updated.connect(self._on_vertical_motor_state_updated)
        self.signal_bridge.servo_motor_state_updated.connect(self._on_horizontal_motor_state_updated)
        self.signal_bridge.motor_work_state_updated.connect(self._on_transport_motor_state_updated)
        self.signal_bridge.distance_sensor_updated.connect(self._on_distance_sensor_updated)
        
        # UI control signals
        self.manual_mode_btn.clicked.connect(self._on_manual_mode_toggled)
        self.set_state_btn.clicked.connect(self._on_set_state_clicked)
        self.confirm_wait_btn.clicked.connect(self._on_confirm_wait_clicked)
        
        self.vertical_motor_panel.set_speed_btn.clicked.connect(self._on_vertical_motor_set_speed)
        self.vertical_motor_panel.set_direction_btn.clicked.connect(self._on_vertical_motor_set_direction)
        
        self.horizontal_motor_panel.set_speed_btn.clicked.connect(self._on_horizontal_motor_set_speed)
        self.horizontal_motor_panel.set_direction_btn.clicked.connect(self._on_horizontal_motor_set_direction)
        # Enable button is hidden but still functional - connect if it exists
        if self.horizontal_motor_panel.enable_btn is not None:
            self.horizontal_motor_panel.enable_btn.clicked.connect(self._on_horizontal_motor_enable)
        
        self.transport_motor_panel.set_speed_btn.clicked.connect(self._on_transport_motor_set_speed)
        self.transport_motor_panel.set_direction_btn.clicked.connect(self._on_transport_motor_set_direction)
        
        # Publish initial manual mode state
        self.ros_node.publish_manual_mode(True)
    
    def _on_state_machine_state_updated(self, state: str):
        self.state_machine_label.setText(state)
        # Important:
        # The combo box is used as a *target state selector* (what user wants to set),
        # not as a display of the current state. If we force-sync it to the current
        # state at 10Hz, the user's selection will be overwritten (often back to INIT)
        # before they can click "Set", causing wrong state to be sent.
        #
        # So we only sync the combo in AUTO mode; in MANUAL mode we keep user's selection.
        if not self._manual_mode:
            index = self.state_combo.findText(state)
            if index >= 0:
                self.state_combo.blockSignals(True)
                self.state_combo.setCurrentIndex(index)
                self.state_combo.blockSignals(False)
        
        # Show/hide confirm button based on state
        if state == "WAIT":
            self.confirm_wait_btn.setVisible(True)
        else:
            self.confirm_wait_btn.setVisible(False)
    
    def _on_microswitch_status_updated(self, status: str):
        pass  # Not used anymore, individual switches are displayed separately
    
    def _update_microswitch_label(self, label, status: str):
        """Update microswitch label with color based on state"""
        if not status or status == "unknown":
            label.setText("unknown")
            label.setStyleSheet("""
                QLabel {
                    background-color: #f0f0f0;
                    color: #000000;
                    padding: 4px 12px;
                    border: 1px solid #a0a0a0;
                    border-radius: 4px;
                    font-size: 12px;
                    font-weight: bold;
                    min-width: 100px;
                }
            """)
        elif status.endswith("_1"):
            # State is 1 - use green
            label.setText(status)
            label.setStyleSheet("""
                QLabel {
                    background-color: #90EE90;
                    color: #000000;
                    padding: 4px 12px;
                    border: 1px solid #00AA00;
                    border-radius: 4px;
                    font-size: 12px;
                    font-weight: bold;
                    min-width: 100px;
                }
            """)
        else:
            # State is 0 - use default gray
            label.setText(status)
            label.setStyleSheet("""
                QLabel {
                    background-color: #f0f0f0;
                    color: #000000;
                    padding: 4px 12px;
                    border: 1px solid #a0a0a0;
                    border-radius: 4px;
                    font-size: 12px;
                    font-weight: bold;
                    min-width: 100px;
                }
            """)
    
    def _on_vertical_max_updated(self, status: str):
        self._update_microswitch_label(self.vertical_max_label, status)
    
    def _on_vertical_min_updated(self, status: str):
        self._update_microswitch_label(self.vertical_min_label, status)
    
    def _on_horizontal_max_updated(self, status: str):
        self._update_microswitch_label(self.horizontal_max_label, status)
    
    def _on_horizontal_min_updated(self, status: str):
        self._update_microswitch_label(self.horizontal_min_label, status)
    
    def _on_vertical_motor_state_updated(self, state_str: str):
        """Parse state string: 'speed,direction'"""
        try:
            parts = state_str.split(',')
            if len(parts) >= 2:
                speed = float(parts[0])
                direction = parts[1]
                self.vertical_motor_panel.update_status(speed, direction)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse vertical motor state: {e}")
    
    def _on_horizontal_motor_state_updated(self, state_str: str):
        """Parse state string: 'speed,direction,enabled'"""
        try:
            parts = state_str.split(',')
            if len(parts) >= 3:
                speed = float(parts[0])
                direction = parts[1]
                enabled = parts[2].lower() == 'true'
                self.horizontal_motor_panel.update_status(speed, direction, enabled)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse horizontal motor state: {e}")
    
    def _on_transport_motor_state_updated(self, state_str: str):
        """Parse state string: 'speed,direction'"""
        try:
            parts = state_str.split(',')
            if len(parts) >= 2:
                speed = float(parts[0])
                direction = parts[1]
                self.transport_motor_panel.update_status(speed, direction)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse transport motor state: {e}")
    
    def _on_manual_mode_toggled(self, checked):
        self._manual_mode = checked
        if checked:
            self.manual_mode_btn.setText("Manual Mode: ON")
            self.status_label.setText("Manual mode enabled - state machine disabled")
            # Red background for manual mode
            self.manual_mode_btn.setStyleSheet("""
                QPushButton {
                    background-color: #CC0000;
                    color: #ffffff;
                    border: 1px solid #AA0000;
                    padding: 4px 12px;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #DD0000; }
            """)
        else:
            self.manual_mode_btn.setText("Auto Mode: ON")
            self.status_label.setText("Auto mode enabled - state machine enabled")
            # Green background for auto mode
            self.manual_mode_btn.setStyleSheet("""
                QPushButton {
                    background-color: #00AA00;
                    color: #ffffff;
                    border: 1px solid #008800;
                    padding: 4px 12px;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #00BB00; }
            """)
        self.ros_node.publish_manual_mode(checked)
    
    def _on_set_state_clicked(self):
        if not self._manual_mode:
            self.status_label.setText("Please enable manual mode first")
            return
        state = self.state_combo.currentText()
        self.ros_node.publish_set_state(state)
        self.status_label.setText(f"State set to: {state}")
    
    def _on_confirm_wait_clicked(self):
        """Handle confirm WAIT button click - transition from WAIT to PRE"""
        self.ros_node.publish_confirm_wait()
        self.status_label.setText("WAIT confirmed, transitioning to PRE...")
    
    def _on_vertical_motor_set_speed(self):
        if not self._manual_mode:
            self.status_label.setText("Please enable manual mode first")
            return
        speed = self.vertical_motor_panel.get_speed()
        self.ros_node.publish_zd_motor_speed(speed)
        self.status_label.setText(f"Vertical Motor speed set to: {speed:.1f} RPM")
    
    def _on_vertical_motor_set_direction(self):
        if not self._manual_mode:
            self.status_label.setText("Please enable manual mode first")
            return
        direction = self.vertical_motor_panel.get_direction()
        self.ros_node.publish_zd_motor_direction(direction)
        self.status_label.setText(f"Vertical Motor direction set to: {direction}")
    
    def _on_horizontal_motor_set_speed(self):
        if not self._manual_mode:
            self.status_label.setText("Please enable manual mode first")
            return
        speed = self.horizontal_motor_panel.get_speed()
        self.ros_node.publish_servo_motor_speed(speed)
        self.status_label.setText(f"Horizontal Motor speed set to: {speed:.1f} RPM")
    
    def _on_horizontal_motor_set_direction(self):
        if not self._manual_mode:
            self.status_label.setText("Please enable manual mode first")
            return
        direction = self.horizontal_motor_panel.get_direction()
        self.ros_node.publish_servo_motor_direction(direction)
        self.status_label.setText(f"Horizontal Motor direction set to: {direction}")
    
    def _on_horizontal_motor_enable(self, checked):
        if not self._manual_mode:
            self.status_label.setText("Please enable manual mode first")
            self.horizontal_motor_panel.enable_btn.blockSignals(True)
            self.horizontal_motor_panel.enable_btn.setChecked(not checked)
            self.horizontal_motor_panel.enable_btn.blockSignals(False)
            return
        self.ros_node.publish_servo_motor_enable(checked)
        self.status_label.setText(f"Horizontal Motor enable set to: {checked}")
    
    def _on_transport_motor_set_speed(self):
        if not self._manual_mode:
            self.status_label.setText("Please enable manual mode first")
            return
        # Note: transport motor manual control not implemented in C++ node yet
        self.status_label.setText("Transport Motor manual control not yet implemented")
    
    def _on_transport_motor_set_direction(self):
        if not self._manual_mode:
            self.status_label.setText("Please enable manual mode first")
            return
        # Note: transport motor manual control not implemented in C++ node yet
        self.status_label.setText("Transport Motor manual control not yet implemented")
    
    def _on_distance_sensor_updated(self, distance: float):
        """Handle distance sensor update"""
        self.distance_plot.update_distance(distance)
    
    def get_logger(self):
        return self.ros_node.get_logger()


# ============================================================================
# Main Entry Point
# ============================================================================
def main():
    """Main entry point"""
    # Initialize ROS2
    rclpy.init()
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Create signal bridge
    signal_bridge = RosSignalBridge()
    
    # Create ROS2 node
    ros_node = TransportWorkflowNode(signal_bridge)
    
    # Create and show GUI
    gui = TransportWorkflowGUI(ros_node, signal_bridge)
    # Use showFullScreen() for better fullscreen experience, or show() for windowed mode
    # gui.showFullScreen()  # Uncomment for fullscreen
    gui.show()
    
    # Create executor for ROS2
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    # Run ROS2 spin in separate thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    # Run Qt event loop
    exit_code = app.exec_()
    
    # Cleanup
    ros_node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
