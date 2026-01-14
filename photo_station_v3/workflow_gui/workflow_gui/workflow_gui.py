#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Photo Station V3 Workflow GUI

Complete workflow GUI integrating:
- Motor control (dual LC servo motors)
- Light control (dual light sources)
- Dual camera image display (front and rear stitched images)

Color scheme: Black/White/Gray theme similar to image_viewer

Topics:
    Motor Control:
        /predictor/front_motor/cmd_vel (Float64): Target speed for front motor (RPM)
        /predictor/rear_motor/cmd_vel (Float64): Target speed for rear motor (RPM)
        /predictor/front_motor/enable (Bool): Enable/disable front motor
        /predictor/rear_motor/enable (Bool): Enable/disable rear motor
        /predictor/front_motor/velocity (Float64): Current speed of front motor (RPM)
        /predictor/rear_motor/velocity (Float64): Current speed of rear motor (RPM)
    
    Light Control:
        light1/control (Bool): Enable/disable light 1
        light2/control (Bool): Enable/disable light 2
        light1/set_brightness (Int32): Set brightness for light 1 (0-255)
        light2/set_brightness (Int32): Set brightness for light 2 (0-255)
        light1/status (Bool): Current status of light 1
        light2/status (Bool): Current status of light 2
        light1/brightness (Int32): Current brightness of light 1
        light2/brightness (Int32): Current brightness of light 2
    
    Image Display:
        /camera_front/front/image_stitched (Image): Front camera stitched image
        /camera_rear/rear/image_stitched (Image): Rear camera stitched image

Services:
    /camera_front/image_stitching_front/reset_stitching (Trigger): Reset front stitching
    /camera_rear/image_stitching_rear/reset_stitching (Trigger): Reset rear stitching
"""

import sys
import os
import warnings

# ============================================================================
# Suppress NumPy 2.x compatibility warnings from compiled modules
# These are just warnings and don't affect functionality
# ============================================================================
warnings.filterwarnings('ignore', message='.*NumPy 1.x.*')
warnings.filterwarnings('ignore', message='.*_ARRAY_API.*')

# ============================================================================
# IMPORTANT: Fix Qt plugin conflict between OpenCV and PyQt5
# Must be done BEFORE importing PyQt5 or cv2
# ============================================================================

# Force using system Qt plugins, not OpenCV's bundled ones
# This prevents conflicts between opencv-python's Qt and PyQt5
_cv2_qt_path = os.path.expanduser('~/.local/lib/python3.10/site-packages/cv2/qt/plugins')
if os.path.exists(_cv2_qt_path):
    # Set QT_PLUGIN_PATH to system path, excluding cv2's path
    os.environ['QT_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins'

# Remove any OpenCV Qt plugin path settings
for var in ['QT_QPA_PLATFORM_PLUGIN_PATH', 'CV2_QT_PLUGIN_PATH']:
    if var in os.environ:
        del os.environ[var]

# Ensure we use xcb platform
if 'QT_QPA_PLATFORM' not in os.environ:
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

import threading
import subprocess
import signal
from typing import Optional, List

# Import numpy first to ensure proper initialization
import numpy as np

# Import PyQt5 BEFORE cv2 to ensure correct Qt bindings
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QGroupBox, QFrame, QTabWidget,
    QDoubleSpinBox, QGridLayout, QSizePolicy, QScrollArea, QSplitter,
    QComboBox, QSpinBox, QMessageBox, QFileDialog, QLineEdit
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QSize, QCoreApplication
from PyQt5.QtGui import QFont, QPixmap, QImage, QPalette

# ============================================================================
# CRITICAL: Initialize QApplication BEFORE importing cv2
# This ensures PyQt5's Qt is used instead of OpenCV's bundled Qt
# ============================================================================
# Set Qt attribute before creating QApplication
QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts, True)

# Patch cv2's Qt plugin path before importing
_cv2_qt_plugin_path = os.path.expanduser('~/.local/lib/python3.10/site-packages/cv2/qt/plugins')
if os.path.isdir(_cv2_qt_plugin_path):
    # Temporarily rename the directory to prevent cv2 from finding it
    import shutil
    _cv2_qt_disabled = _cv2_qt_plugin_path + '.disabled'
    if not os.path.exists(_cv2_qt_disabled):
        try:
            shutil.move(_cv2_qt_plugin_path, _cv2_qt_disabled)
        except (PermissionError, OSError):
            pass  # Can't rename, will try other methods

# Now import OpenCV (headless operations only, Qt handled by PyQt5)
import cv2

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64, Bool, Int32, String, Float32
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

# cv_bridge will be imported lazily to avoid NumPy 2.x compatibility issues
CV_BRIDGE_AVAILABLE = False
CvBridge = None


def try_import_cv_bridge():
    """
    Try to import cv_bridge lazily.
    Returns True if successful, False otherwise.
    Tests that cv_bridge actually works by creating an instance.
    """
    global CV_BRIDGE_AVAILABLE, CvBridge
    if CV_BRIDGE_AVAILABLE:
        return True
    try:
        from cv_bridge import CvBridge as _CvBridge
        # Test that we can actually instantiate CvBridge
        test_bridge = _CvBridge()
        # Test a simple operation to ensure it works
        import numpy as np
        test_img = np.zeros((10, 10), dtype=np.uint8)
        # If we get here, cv_bridge is working
        CvBridge = _CvBridge
        CV_BRIDGE_AVAILABLE = True
        return True
    except Exception as e:
        print(f"Warning: cv_bridge not available ({e}), using manual image conversion")
        CV_BRIDGE_AVAILABLE = False
        CvBridge = None
        return False


# ============================================================================
# Manual Image Conversion (Fallback when cv_bridge is not available)
# ============================================================================
def imgmsg_to_cv2_manual(msg: Image) -> np.ndarray:
    """
    Convert ROS2 Image message to OpenCV image without cv_bridge.
    Supports common encodings: mono8, bgr8, rgb8, rgba8, bgra8
    """
    encoding = msg.encoding.lower()
    
    # Determine dtype and channels
    if encoding in ['mono8', '8uc1']:
        dtype = np.uint8
        channels = 1
    elif encoding in ['bgr8', 'rgb8', '8uc3']:
        dtype = np.uint8
        channels = 3
    elif encoding in ['bgra8', 'rgba8', '8uc4']:
        dtype = np.uint8
        channels = 4
    elif encoding in ['mono16', '16uc1']:
        dtype = np.uint16
        channels = 1
    elif encoding in ['32fc1']:
        dtype = np.float32
        channels = 1
    else:
        # Default to mono8
        dtype = np.uint8
        channels = 1
    
    # Create numpy array from raw data
    if channels == 1:
        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
    else:
        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
    
    # Convert RGB to BGR if needed (OpenCV uses BGR)
    if encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif encoding == 'rgba8':
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA)
    
    return img.copy()  # Return a copy to ensure data is contiguous


# ============================================================================
# ROS Signal Bridge for Thread-Safe Qt Updates
# ============================================================================
class RosSignalBridge(QObject):
    """Bridge to safely emit Qt signals from ROS callbacks"""
    # Motor signals
    front_velocity_updated = pyqtSignal(float)
    rear_velocity_updated = pyqtSignal(float)
    
    # Light signals
    light1_status_updated = pyqtSignal(bool)
    light2_status_updated = pyqtSignal(bool)
    light1_brightness_updated = pyqtSignal(int)
    light2_brightness_updated = pyqtSignal(int)
    light1_voltage_updated = pyqtSignal(float)
    light1_current_updated = pyqtSignal(float)
    light2_voltage_updated = pyqtSignal(float)
    light2_current_updated = pyqtSignal(float)
    connection_status_updated = pyqtSignal(bool)
    
    # Image signals
    front_image_updated = pyqtSignal(object)
    rear_image_updated = pyqtSignal(object)


# ============================================================================
# ROS2 Node
# ============================================================================
class WorkflowNode(Node):
    """ROS2 Node for workflow GUI"""
    
    def __init__(self, signal_bridge: RosSignalBridge):
        super().__init__('workflow_gui')
        self.signal_bridge = signal_bridge
        
        # Try to initialize cv_bridge (lazy import to avoid NumPy 2.x issues)
        if try_import_cv_bridge():
            self.cv_bridge = CvBridge()
            self.get_logger().info('Using cv_bridge for image conversion')
        else:
            self.cv_bridge = None
            self.get_logger().warn('Using manual image conversion (cv_bridge not available)')
        
        self._setup_motor_interfaces()
        self._setup_light_interfaces()
        self._setup_image_interfaces()
        self._setup_services()
        
        self.get_logger().info('Workflow GUI Node initialized')
    
    def _setup_motor_interfaces(self):
        """Setup motor control publishers and subscribers"""
        # Publishers
        self.front_cmd_pub = self.create_publisher(
            Float64, '/predictor/front_motor/cmd_vel', 10)
        self.rear_cmd_pub = self.create_publisher(
            Float64, '/predictor/rear_motor/cmd_vel', 10)
        self.front_enable_pub = self.create_publisher(
            Bool, '/predictor/front_motor/enable', 10)
        self.rear_enable_pub = self.create_publisher(
            Bool, '/predictor/rear_motor/enable', 10)
        
        # Subscribers
        self.front_vel_sub = self.create_subscription(
            Float64, '/predictor/front_motor/velocity',
            self._front_velocity_callback, 10)
        self.rear_vel_sub = self.create_subscription(
            Float64, '/predictor/rear_motor/velocity',
            self._rear_velocity_callback, 10)
    
    def _setup_light_interfaces(self):
        """Setup light control publishers and subscribers"""
        # Publishers
        self.light1_control_pub = self.create_publisher(
            Bool, 'light1/control', 10)
        self.light2_control_pub = self.create_publisher(
            Bool, 'light2/control', 10)
        self.light1_brightness_pub = self.create_publisher(
            Int32, 'light1/set_brightness', 10)
        self.light2_brightness_pub = self.create_publisher(
            Int32, 'light2/set_brightness', 10)
        
        # Subscribers
        self.light1_status_sub = self.create_subscription(
            Bool, 'light1/status', self._light1_status_callback, 10)
        self.light2_status_sub = self.create_subscription(
            Bool, 'light2/status', self._light2_status_callback, 10)
        self.light1_brightness_sub = self.create_subscription(
            Int32, 'light1/brightness', self._light1_brightness_callback, 10)
        self.light2_brightness_sub = self.create_subscription(
            Int32, 'light2/brightness', self._light2_brightness_callback, 10)
        self.light1_voltage_sub = self.create_subscription(
            Float32, 'light1/voltage', self._light1_voltage_callback, 10)
        self.light1_current_sub = self.create_subscription(
            Float32, 'light1/current', self._light1_current_callback, 10)
        self.light2_voltage_sub = self.create_subscription(
            Float32, 'light2/voltage', self._light2_voltage_callback, 10)
        self.light2_current_sub = self.create_subscription(
            Float32, 'light2/current', self._light2_current_callback, 10)
        self.connection_status_sub = self.create_subscription(
            Bool, 'connection_status', self._connection_status_callback, 10)
    
    def _setup_image_interfaces(self):
        """Setup image subscribers - initially None, will be set dynamically"""
        self.front_image_sub = None
        self.rear_image_sub = None
        self._front_topic = ""
        self._rear_topic = ""
    
    def get_available_image_topics(self) -> list:
        """Get list of available sensor_msgs/msg/Image topics"""
        topics = []
        try:
            topic_list = self.get_topic_names_and_types()
            for name, types in topic_list:
                for t in types:
                    if t == "sensor_msgs/msg/Image":
                        topics.append(name)
        except Exception as e:
            self.get_logger().error(f"Failed to get topics: {e}")
        return sorted(topics)
    
    def subscribe_front_image(self, topic: str):
        """Subscribe to a new front image topic"""
        if topic == self._front_topic:
            return
        
        # Unsubscribe from previous topic
        if self.front_image_sub is not None:
            self.destroy_subscription(self.front_image_sub)
            self.front_image_sub = None
        
        if not topic:
            self._front_topic = ""
            return
        
        try:
            self.front_image_sub = self.create_subscription(
                Image, topic, self._front_image_callback, 10)
            self._front_topic = topic
            self.get_logger().info(f"Subscribed to front image topic: {topic}")
        except Exception as e:
            self.get_logger().error(f"Failed to subscribe to {topic}: {e}")
    
    def subscribe_rear_image(self, topic: str):
        """Subscribe to a new rear image topic"""
        if topic == self._rear_topic:
            return
        
        # Unsubscribe from previous topic
        if self.rear_image_sub is not None:
            self.destroy_subscription(self.rear_image_sub)
            self.rear_image_sub = None
        
        if not topic:
            self._rear_topic = ""
            return
        
        try:
            self.rear_image_sub = self.create_subscription(
                Image, topic, self._rear_image_callback, 10)
            self._rear_topic = topic
            self.get_logger().info(f"Subscribed to rear image topic: {topic}")
        except Exception as e:
            self.get_logger().error(f"Failed to subscribe to {topic}: {e}")
    
    def _setup_services(self):
        """Setup service clients"""
        self.front_reset_client = self.create_client(
            Trigger, '/camera_front/image_stitching_front/reset_stitching')
        self.rear_reset_client = self.create_client(
            Trigger, '/camera_rear/image_stitching_rear/reset_stitching')
    
    # Motor callbacks
    def _front_velocity_callback(self, msg: Float64):
        self.signal_bridge.front_velocity_updated.emit(msg.data)
    
    def _rear_velocity_callback(self, msg: Float64):
        self.signal_bridge.rear_velocity_updated.emit(msg.data)
    
    # Light callbacks
    def _light1_status_callback(self, msg: Bool):
        self.signal_bridge.light1_status_updated.emit(msg.data)
    
    def _light2_status_callback(self, msg: Bool):
        self.signal_bridge.light2_status_updated.emit(msg.data)
    
    def _light1_brightness_callback(self, msg: Int32):
        self.signal_bridge.light1_brightness_updated.emit(msg.data)
    
    def _light2_brightness_callback(self, msg: Int32):
        self.signal_bridge.light2_brightness_updated.emit(msg.data)
    
    def _light1_voltage_callback(self, msg: Float32):
        self.signal_bridge.light1_voltage_updated.emit(msg.data)
    
    def _light1_current_callback(self, msg: Float32):
        self.signal_bridge.light1_current_updated.emit(msg.data)
    
    def _light2_voltage_callback(self, msg: Float32):
        self.signal_bridge.light2_voltage_updated.emit(msg.data)
    
    def _light2_current_callback(self, msg: Float32):
        self.signal_bridge.light2_current_updated.emit(msg.data)
    
    def _connection_status_callback(self, msg: Bool):
        self.signal_bridge.connection_status_updated.emit(msg.data)
    
    # Image callbacks
    def _front_image_callback(self, msg: Image):
        try:
            if self.cv_bridge is not None:
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            else:
                cv_image = imgmsg_to_cv2_manual(msg)
            self.signal_bridge.front_image_updated.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f'Front image conversion error: {e}')

    def _rear_image_callback(self, msg: Image):
        try:
            if self.cv_bridge is not None:
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            else:
                cv_image = imgmsg_to_cv2_manual(msg)
            self.signal_bridge.rear_image_updated.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f'Rear image conversion error: {e}')
    
    # Motor control methods
    def publish_front_speed(self, speed: float):
        msg = Float64()
        msg.data = speed
        self.front_cmd_pub.publish(msg)
    
    def publish_rear_speed(self, speed: float):
        msg = Float64()
        msg.data = speed
        self.rear_cmd_pub.publish(msg)
    
    def publish_front_enable(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.front_enable_pub.publish(msg)
    
    def publish_rear_enable(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.rear_enable_pub.publish(msg)
    
    # Light control methods
    def publish_light1_control(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.light1_control_pub.publish(msg)
    
    def publish_light2_control(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.light2_control_pub.publish(msg)
    
    def publish_light1_brightness(self, brightness: int):
        msg = Int32()
        msg.data = brightness
        self.light1_brightness_pub.publish(msg)
    
    def publish_light2_brightness(self, brightness: int):
        msg = Int32()
        msg.data = brightness
        self.light2_brightness_pub.publish(msg)
    
    # Service methods
    def reset_front_stitching(self, callback):
        if not self.front_reset_client.wait_for_service(timeout_sec=1.0):
            return False
        request = Trigger.Request()
        future = self.front_reset_client.call_async(request)
        future.add_done_callback(callback)
        return True
    
    def reset_rear_stitching(self, callback):
        if not self.rear_reset_client.wait_for_service(timeout_sec=1.0):
            return False
        request = Trigger.Request()
        future = self.rear_reset_client.call_async(request)
        future.add_done_callback(callback)
        return True


# ============================================================================
# Motor Control Panel Widget
# ============================================================================
class MotorControlPanel(QGroupBox):
    """Control panel for a single motor"""
    
    def __init__(self, motor_name: str, parent=None):
        super().__init__(motor_name, parent)
        self.motor_name = motor_name
        self._enabled = False
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout()
        layout.setSpacing(8)
        
        # Status indicator
        status_layout = QHBoxLayout()
        self.status_label = QLabel("Status: Disabled")
        self.status_label.setStyleSheet("color: #808080; font-weight: bold;")
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        layout.addLayout(status_layout)
        
        # Enable/Disable buttons
        btn_layout = QHBoxLayout()
        self.enable_btn = QPushButton("Enable")
        self.enable_btn.setStyleSheet("""
            QPushButton {
                background-color: #E0E0E0;
                color: #000000;
                border: 1px solid #808080;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #D0D0D0; }
            QPushButton:pressed { background-color: #C0C0C0; }
        """)
        
        self.disable_btn = QPushButton("Disable")
        self.disable_btn.setStyleSheet("""
            QPushButton {
                background-color: #606060;
                color: #ffffff;
                border: 1px solid #404040;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #707070; }
            QPushButton:pressed { background-color: #505050; }
        """)
        
        btn_layout.addWidget(self.enable_btn)
        btn_layout.addWidget(self.disable_btn)
        layout.addLayout(btn_layout)
        
        # Separator
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setStyleSheet("background-color: #c0c0c0;")
        layout.addWidget(line)
        
        # Speed control section
        speed_label = QLabel("Speed Control (RPM):")
        speed_label.setFont(QFont("Arial", 9, QFont.Bold))
        layout.addWidget(speed_label)
        
        # Speed slider
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(-3000)
        self.speed_slider.setMaximum(3000)
        self.speed_slider.setValue(0)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(500)
        layout.addWidget(self.speed_slider)
        
        # Speed input and buttons
        input_layout = QHBoxLayout()
        
        self.speed_spinbox = QDoubleSpinBox()
        self.speed_spinbox.setMinimum(-3000)
        self.speed_spinbox.setMaximum(3000)
        self.speed_spinbox.setValue(0)
        self.speed_spinbox.setSuffix(" RPM")
        self.speed_spinbox.setDecimals(1)
        self.speed_spinbox.setSingleStep(10)
        
        self.set_speed_btn = QPushButton("Set")
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
            QPushButton:pressed { background-color: #707070; }
        """)
        
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #404040;
                color: #ffffff;
                border: 1px solid #202020;
                padding: 6px 12px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #505050; }
            QPushButton:pressed { background-color: #303030; }
        """)
        
        input_layout.addWidget(self.speed_spinbox)
        input_layout.addWidget(self.set_speed_btn)
        input_layout.addWidget(self.stop_btn)
        layout.addLayout(input_layout)
        
        # Current velocity display
        vel_layout = QHBoxLayout()
        vel_label = QLabel("Current Speed:")
        self.velocity_display = QLabel("0.00 RPM")
        self.velocity_display.setStyleSheet("""
            QLabel {
                background-color: #f0f0f0;
                color: #000000;
                padding: 8px 16px;
                border: 1px solid #a0a0a0;
                border-radius: 4px;
                font-size: 14px;
                font-weight: bold;
                font-family: 'Courier New', monospace;
            }
        """)
        self.velocity_display.setAlignment(Qt.AlignCenter)
        vel_layout.addWidget(vel_label)
        vel_layout.addWidget(self.velocity_display)
        layout.addLayout(vel_layout)
        
        self.setLayout(layout)
        
        # Connect internal signals
        self.speed_slider.valueChanged.connect(self._on_slider_changed)
        self.speed_spinbox.valueChanged.connect(self._on_spinbox_changed)
        self.stop_btn.clicked.connect(self._on_stop_clicked)
    
    def _on_slider_changed(self, value):
        self.speed_spinbox.blockSignals(True)
        self.speed_spinbox.setValue(value)
        self.speed_spinbox.blockSignals(False)
    
    def _on_spinbox_changed(self, value):
        self.speed_slider.blockSignals(True)
        self.speed_slider.setValue(int(value))
        self.speed_slider.blockSignals(False)
    
    def _on_stop_clicked(self):
        self.speed_slider.setValue(0)
        self.speed_spinbox.setValue(0)
        # Immediately publish speed=0 using existing Set logic
        self.set_speed_btn.click()
    
    def update_velocity(self, velocity: float):
        self.velocity_display.setText(f"{velocity:.2f} RPM")
    
    def set_enabled_state(self, enabled: bool):
        self._enabled = enabled
        if enabled:
            self.status_label.setText("Status: Enabled")
            self.status_label.setStyleSheet("color: #008000; font-weight: bold;")
        else:
            self.status_label.setText("Status: Disabled")
            self.status_label.setStyleSheet("color: #808080; font-weight: bold;")
    
    def get_speed(self) -> float:
        return self.speed_spinbox.value()


# ============================================================================
# Light Control Panel Widget
# ============================================================================
class LightControlPanel(QGroupBox):
    """Control panel for a single light source"""
    
    def __init__(self, light_name: str, parent=None):
        super().__init__(light_name, parent)
        self.light_name = light_name
        self._enabled = False
        self._brightness = 0
        self._voltage = 0.0
        self._current = 0.0
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout()
        layout.setSpacing(8)
        
        # Toggle button
        self.toggle_btn = QPushButton("OFF")
        self.toggle_btn.setStyleSheet("""
            QPushButton {
                font-size: 12pt;
                padding: 10px;
                background-color: #606060;
                color: #ffffff;
                border: 1px solid #404040;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #707070; }
        """)
        layout.addWidget(self.toggle_btn)
        
        # Status display
        status_layout = QGridLayout()
        
        self.status_label = QLabel("Status: OFF")
        status_layout.addWidget(self.status_label, 0, 0)
        
        self.brightness_label = QLabel("Brightness: 0")
        status_layout.addWidget(self.brightness_label, 0, 1)
        
        self.voltage_label = QLabel("Voltage: 0.00 V")
        status_layout.addWidget(self.voltage_label, 1, 0)
        
        self.current_label = QLabel("Current: 0.00 A")
        status_layout.addWidget(self.current_label, 1, 1)
        
        self.power_label = QLabel("Power: 0.00 W")
        self.power_label.setStyleSheet("color: #0066aa; font-weight: bold;")
        status_layout.addWidget(self.power_label, 2, 0, 1, 2)
        
        layout.addLayout(status_layout)
        
        # Brightness slider
        brightness_layout = QHBoxLayout()
        brightness_layout.addWidget(QLabel("Brightness:"))
        
        self.brightness_slider = QSlider(Qt.Horizontal)
        self.brightness_slider.setRange(0, 255)
        self.brightness_slider.setValue(0)
        brightness_layout.addWidget(self.brightness_slider)
        
        self.brightness_spinbox = QSpinBox()
        self.brightness_spinbox.setRange(0, 255)
        self.brightness_spinbox.setValue(0)
        brightness_layout.addWidget(self.brightness_spinbox)
        
        layout.addLayout(brightness_layout)
        
        self.setLayout(layout)
        
        # Connect internal signals
        self.brightness_slider.valueChanged.connect(self._on_slider_changed)
        self.brightness_spinbox.valueChanged.connect(self._on_spinbox_changed)
    
    def _on_slider_changed(self, value):
        self.brightness_spinbox.blockSignals(True)
        self.brightness_spinbox.setValue(value)
        self.brightness_spinbox.blockSignals(False)
    
    def _on_spinbox_changed(self, value):
        self.brightness_slider.blockSignals(True)
        self.brightness_slider.setValue(value)
        self.brightness_slider.blockSignals(False)
    
    def update_status(self, enabled: bool):
        self._enabled = enabled
        if enabled:
            self.toggle_btn.setText("ON")
            self.toggle_btn.setStyleSheet("""
                QPushButton {
                    font-size: 12pt;
                    padding: 10px;
                    background-color: #808080;
                    color: #ffffff;
                    border: 1px solid #606060;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #909090; }
            """)
            self.status_label.setText("Status: ON")
        else:
            self.toggle_btn.setText("OFF")
            self.toggle_btn.setStyleSheet("""
                QPushButton {
                    font-size: 12pt;
                    padding: 10px;
                    background-color: #606060;
                    color: #ffffff;
                    border: 1px solid #404040;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #707070; }
            """)
            self.status_label.setText("Status: OFF")
    
    def update_brightness(self, brightness: int):
        self._brightness = brightness
        self.brightness_slider.blockSignals(True)
        self.brightness_slider.setValue(brightness)
        self.brightness_slider.blockSignals(False)
        self.brightness_spinbox.blockSignals(True)
        self.brightness_spinbox.setValue(brightness)
        self.brightness_spinbox.blockSignals(False)
        self.brightness_label.setText(f"Brightness: {brightness}")
    
    def update_voltage(self, voltage: float):
        self._voltage = voltage
        self.voltage_label.setText(f"Voltage: {voltage:.2f} V")
        self._update_power()
    
    def update_current(self, current: float):
        self._current = current
        self.current_label.setText(f"Current: {current:.2f} A")
        self._update_power()
    
    def _update_power(self):
        power = self._voltage * self._current
        self.power_label.setText(f"Power: {power:.2f} W")
    
    def get_brightness(self) -> int:
        return self.brightness_spinbox.value()
    
    def is_enabled(self) -> bool:
        return self._enabled


# ============================================================================
# Image Display Panel Widget
# ============================================================================
class ImageDisplayPanel(QGroupBox):
    """Panel for displaying camera image"""
    
    # Signal for topic change request
    topic_changed = pyqtSignal(str)
    refresh_topics_requested = pyqtSignal()
    
    def __init__(self, title: str, default_topic: str = "", parent=None):
        super().__init__(title, parent)
        self._current_image = None
        self._frame_count = 0
        self._save_count = 0
        self._default_topic = default_topic
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout()
        layout.setSpacing(5)
        
        # Topic selection bar
        topic_layout = QHBoxLayout()
        
        topic_label = QLabel("Topic:")
        topic_layout.addWidget(topic_label)
        
        self.topic_combo = QComboBox()
        self.topic_combo.setEditable(True)
        self.topic_combo.setMinimumWidth(500)
        self.topic_combo.lineEdit().setPlaceholderText("/image_topic")
        if self._default_topic:
            self.topic_combo.setCurrentText(self._default_topic)
        self.topic_combo.currentTextChanged.connect(self._on_topic_changed)
        topic_layout.addWidget(self.topic_combo)
        
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.setMaximumWidth(80)
        self.refresh_btn.setStyleSheet("""
            QPushButton {
                background-color: #E0E0E0;
                border: 1px solid #808080;
                border-radius: 3px;
                padding: 5px;
            }
            QPushButton:hover { background-color: #D0D0D0; }
            QPushButton:pressed { background-color: #C0C0C0; }
        """)
        self.refresh_btn.clicked.connect(self._on_refresh_clicked)
        topic_layout.addWidget(self.refresh_btn)
        
        topic_layout.addStretch()
        layout.addLayout(topic_layout)
        
        # Control bar
        control_layout = QHBoxLayout()
        
        self.reset_btn = QPushButton("Reset")
        self.reset_btn.setStyleSheet("""
            QPushButton {
                background-color: #E0E0E0;
                border: 1px solid #808080;
                border-radius: 3px;
                padding: 5px 10px;
            }
            QPushButton:hover { background-color: #D0D0D0; }
            QPushButton:pressed { background-color: #C0C0C0; }
        """)
        control_layout.addWidget(self.reset_btn)
        
        control_layout.addStretch()
        
        self.info_label = QLabel("Waiting for image...")
        self.info_label.setStyleSheet("color: #606060;")
        control_layout.addWidget(self.info_label)
        
        layout.addLayout(control_layout)
        
        # Save path bar (similar to Topic bar)
        save_layout = QHBoxLayout()
        
        save_path_label = QLabel("Save Path:")
        save_layout.addWidget(save_path_label)
        
        from datetime import datetime
        default_save_path = f"image_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        self.save_path_edit = QLineEdit()
        self.save_path_edit.setMinimumWidth(500)
        self.save_path_edit.setText(default_save_path)
        self.save_path_edit.setPlaceholderText("Enter save path...")
        save_layout.addWidget(self.save_path_edit)
        
        self.save_btn = QPushButton("Save")
        self.save_btn.setMaximumWidth(80)
        self.save_btn.setStyleSheet("""
            QPushButton {
                background-color: #E0E0E0;
                border: 1px solid #808080;
                border-radius: 3px;
                padding: 5px;
            }
            QPushButton:hover { background-color: #D0D0D0; }
            QPushButton:pressed { background-color: #C0C0C0; }
        """)
        self.save_btn.clicked.connect(self._on_save_clicked)
        save_layout.addWidget(self.save_btn)
        
        self.save_count_label = QLabel("Saved: 0")
        self.save_count_label.setStyleSheet("color: #606060; font-weight: bold;")
        save_layout.addWidget(self.save_count_label)
        
        save_layout.addStretch()
        layout.addLayout(save_layout)
        
        # Scroll area for image
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setBackgroundRole(QPalette.Light)
        self.scroll_area.setAlignment(Qt.AlignCenter)
        self.scroll_area.setStyleSheet("""
            QScrollArea {
                background-color: #ffffff;
                border: 1px solid #c0c0c0;
            }
        """)
        
        self.image_label = QLabel("Waiting for image...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(320, 240)
        self.image_label.setStyleSheet("color: #606060; background-color: #ffffff;")
        
        self.scroll_area.setWidget(self.image_label)
        layout.addWidget(self.scroll_area, 1)
        
        self.setLayout(layout)
    
    def _on_topic_changed(self, topic: str):
        """Handle topic change from combo box"""
        if topic.strip():
            self.topic_changed.emit(topic.strip())
    
    def _on_refresh_clicked(self):
        """Handle refresh button click"""
        self.refresh_topics_requested.emit()
    
    def update_topic_list(self, topics: list):
        """Update the topic combo box with available topics"""
        current = self.topic_combo.currentText()
        self.topic_combo.blockSignals(True)
        self.topic_combo.clear()
        for topic in sorted(topics):
            self.topic_combo.addItem(topic)
        if current:
            self.topic_combo.setCurrentText(current)
        self.topic_combo.blockSignals(False)
    
    def get_current_topic(self) -> str:
        """Get the currently selected topic"""
        return self.topic_combo.currentText().strip()
    
    def update_image(self, cv_image):
        """Update displayed image from OpenCV image"""
        if cv_image is None:
            return
        
        self._current_image = cv_image.copy()
        self._frame_count += 1
        
        # Convert to QImage
        if len(cv_image.shape) == 2:
            # Grayscale
            height, width = cv_image.shape
            bytes_per_line = width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_Grayscale8)
        elif cv_image.shape[2] == 3:
            # BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            height, width, channel = rgb_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        else:
            return
        
        # Scale to fit while maintaining aspect ratio
        pixmap = QPixmap.fromImage(q_image)
        scaled_pixmap = pixmap.scaled(
            self.scroll_area.size() - QSize(20, 20),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        
        self.image_label.setPixmap(scaled_pixmap)
        self.info_label.setText(f"Size: {width}x{height} | Frames: {self._frame_count}")
    
    def clear_image(self):
        """Clear the displayed image"""
        self._current_image = None
        self._frame_count = 0
        self.image_label.clear()
        self.image_label.setText("Waiting for image...")
        self.info_label.setText("Waiting for image...")
    
    def _on_save_clicked(self):
        """Save current image to file"""
        if self._current_image is None:
            return
        
        filename = self.save_path_edit.text().strip()
        if not filename:
            return
        
        try:
            # Ensure directory exists
            import os
            save_dir = os.path.dirname(filename)
            if save_dir and not os.path.exists(save_dir):
                os.makedirs(save_dir, exist_ok=True)
            
            cv2.imwrite(filename, self._current_image)
            self._save_count += 1
            self.save_count_label.setText(f"Saved: {self._save_count}")
            
            # Update default path for next save
            from datetime import datetime
            default_save_path = f"image_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            self.save_path_edit.setText(default_save_path)
        except Exception as e:
            self.save_count_label.setText(f"Save failed: {str(e)[:20]}")


# ============================================================================
# Main Workflow GUI Window
# ============================================================================
class WorkflowGUI(QMainWindow):
    """Main workflow GUI window"""
    
    def __init__(self, ros_node: WorkflowNode, signal_bridge: RosSignalBridge):
        super().__init__()
        self.ros_node = ros_node
        self.signal_bridge = signal_bridge
        
        self._front_motor_enabled = False
        self._rear_motor_enabled = False
        self._light1_enabled = False
        self._light2_enabled = False
        
        self._setup_ui()
        self._connect_signals()
    
    def _setup_ui(self):
        self.setWindowTitle("Photo Station V3 - Workflow Control")
        self.setMinimumSize(1400, 900)
        
        # Apply light gray/white theme (similar to image_viewer)
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
            QSlider::handle:horizontal:hover {
                background: #505050;
            }
            QDoubleSpinBox, QSpinBox {
                background-color: #ffffff;
                color: #303030;
                border: 1px solid #a0a0a0;
                border-radius: 4px;
                padding: 4px;
            }
            QDoubleSpinBox:focus, QSpinBox:focus {
                border: 2px solid #606060;
            }
            QSpinBox::up-button, QDoubleSpinBox::up-button {
                subcontrol-origin: border;
                subcontrol-position: top right;
                width: 18px;
                background-color: #e0e0e0;
                border-left: 1px solid #808080;
                border-bottom: 1px solid #808080;
                border-top-right-radius: 4px;
            }
            QSpinBox::down-button, QDoubleSpinBox::down-button {
                subcontrol-origin: border;
                subcontrol-position: bottom right;
                width: 18px;
                background-color: #e0e0e0;
                border-left: 1px solid #808080;
                border-bottom-right-radius: 4px;
            }
            QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
            QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {
                background-color: #d0d0d0;
            }
            QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {
                width: 8px;
                height: 8px;
                image: url("data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='8' height='8' viewBox='0 0 8 8'><path d='M1.5 5.5 L4 2.5 L6.5 5.5 Z' fill='%23404040' stroke='none'/></svg>");
            }
            QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {
                width: 8px;
                height: 8px;
                image: url("data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='8' height='8' viewBox='0 0 8 8'><path d='M1.5 2.5 L4 5.5 L6.5 2.5 Z' fill='%23404040' stroke='none'/></svg>");
            }
            QComboBox {
                background-color: #ffffff;
                color: #303030;
                border: 1px solid #a0a0a0;
                border-radius: 4px;
                padding: 4px;
            }
            QComboBox:focus {
                border: 2px solid #606060;
            }
            QComboBox::drop-down {
                subcontrol-origin: padding;
                subcontrol-position: top right;
                width: 22px;
                border-left: 1px solid #808080;
                background-color: #e0e0e0;
                border-top-right-radius: 4px;
                border-bottom-right-radius: 4px;
            }
            QComboBox::drop-down:hover {
                background-color: #d0d0d0;
            }
            QComboBox::down-arrow {
                width: 8px;
                height: 8px;
                image: url("data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='8' height='8' viewBox='0 0 8 8'><path d='M1.5 2.5 L4 5.5 L6.5 2.5 Z' fill='%23404040' stroke='none'/></svg>");
            }
            QComboBox QAbstractItemView {
                background-color: #ffffff;
                color: #303030;
                selection-background-color: #c0c0c0;
            }
            QLineEdit {
                background-color: #ffffff;
                color: #303030;
                border: 1px solid #a0a0a0;
                border-radius: 4px;
                padding: 4px;
            }
            QTabWidget::pane {
                border: 1px solid #c0c0c0;
                background-color: #ffffff;
            }
            QTabBar::tab {
                background-color: #e0e0e0;
                color: #303030;
                padding: 8px 16px;
                border: 1px solid #c0c0c0;
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background-color: #ffffff;
            }
            QTabBar::tab:hover {
                background-color: #d0d0d0;
            }
            QScrollBar:vertical {
                background-color: #e0e0e0;
                width: 12px;
                border-radius: 6px;
            }
            QScrollBar::handle:vertical {
                background-color: #a0a0a0;
                border-radius: 6px;
                min-height: 20px;
            }
            QScrollBar::handle:vertical:hover {
                background-color: #808080;
            }
            QScrollBar:horizontal {
                background-color: #e0e0e0;
                height: 12px;
                border-radius: 6px;
            }
            QScrollBar::handle:horizontal {
                background-color: #a0a0a0;
                border-radius: 6px;
                min-width: 20px;
            }
            QScrollBar::handle:horizontal:hover {
                background-color: #808080;
            }
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # Title
        title = QLabel("Photo Station V3 - Workflow Control System")
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
        
        # Main splitter
        splitter = QSplitter(Qt.Horizontal)
        
        # Left panel - Controls
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(10)
        
        # Motor control section
        motor_group = QGroupBox("Motor Control")
        motor_layout = QVBoxLayout(motor_group)
        
        motors_h_layout = QHBoxLayout()
        self.front_motor_panel = MotorControlPanel("Front Motor")
        self.rear_motor_panel = MotorControlPanel("Rear Motor")
        motors_h_layout.addWidget(self.front_motor_panel)
        motors_h_layout.addWidget(self.rear_motor_panel)
        motor_layout.addLayout(motors_h_layout)
        
        # Sync control
        sync_layout = QHBoxLayout()
        
        self.sync_speed_spinbox = QDoubleSpinBox()
        self.sync_speed_spinbox.setMinimum(-3000)
        self.sync_speed_spinbox.setMaximum(3000)
        self.sync_speed_spinbox.setValue(0)
        self.sync_speed_spinbox.setSuffix(" RPM")
        
        self.sync_enable_btn = QPushButton("Enable Both")
        self.sync_enable_btn.setStyleSheet("""
            QPushButton {
                background-color: #E0E0E0;
                color: #000000;
                border: 1px solid #808080;
                padding: 6px 12px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #D0D0D0; }
        """)
        
        self.sync_disable_btn = QPushButton("Disable Both")
        self.sync_disable_btn.setStyleSheet("""
            QPushButton {
                background-color: #606060;
                color: #ffffff;
                border: 1px solid #404040;
                padding: 6px 12px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #707070; }
        """)
        
        self.sync_set_btn = QPushButton("Set Both")
        self.sync_set_btn.setStyleSheet("""
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
        
        self.emergency_stop_btn = QPushButton("STOP")
        self.emergency_stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #404040;
                color: #ffffff;
                border: 1px solid #202020;
                padding: 6px 16px;
                border-radius: 4px;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover { background-color: #505050; }
        """)
        
        sync_layout.addWidget(QLabel("Sync Speed:"))
        sync_layout.addWidget(self.sync_speed_spinbox)
        sync_layout.addWidget(self.sync_set_btn)
        sync_layout.addWidget(self.sync_enable_btn)
        sync_layout.addWidget(self.sync_disable_btn)
        sync_layout.addStretch()
        sync_layout.addWidget(self.emergency_stop_btn)
        
        motor_layout.addLayout(sync_layout)
        left_layout.addWidget(motor_group)
        
        # Light control section
        light_group = QGroupBox("Light Control")
        light_layout = QVBoxLayout(light_group)
        
        lights_h_layout = QHBoxLayout()
        self.light1_panel = LightControlPanel("Light 1 (Front)")
        self.light2_panel = LightControlPanel("Light 2 (Rear)")
        lights_h_layout.addWidget(self.light1_panel)
        lights_h_layout.addWidget(self.light2_panel)
        light_layout.addLayout(lights_h_layout)
        
        # Connection status
        self.light_connection_label = QLabel("Light Controller: Disconnected")
        self.light_connection_label.setStyleSheet("color: #aa0000; font-weight: bold;")
        light_layout.addWidget(self.light_connection_label)
        
        left_layout.addWidget(light_group)
        
        splitter.addWidget(left_panel)
        
        # Right panel - Image Display
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setSpacing(10)
        
        image_group = QGroupBox("Image Display")
        image_layout = QVBoxLayout(image_group)
        
        # Image panels in vertical splitter
        image_splitter = QSplitter(Qt.Vertical)
        
        self.front_image_panel = ImageDisplayPanel(
            "Front Camera", "/camera_front/front/image_stitched")
        self.rear_image_panel = ImageDisplayPanel(
            "Rear Camera", "/camera_rear/rear/image_stitched")
        
        image_splitter.addWidget(self.front_image_panel)
        image_splitter.addWidget(self.rear_image_panel)
        
        image_layout.addWidget(image_splitter)
        right_layout.addWidget(image_group)
        
        splitter.addWidget(right_panel)
        
        # Set splitter proportions
        splitter.setSizes([500, 900])
        
        main_layout.addWidget(splitter, 1)
        
        # Status bar
        status_layout = QHBoxLayout()
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("background-color: #f0f0f0; color: #606060; font-size: 10px; padding: 3px;")
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        
        main_layout.addLayout(status_layout)
        
        central_widget.setLayout(main_layout)
    
    def init_topics(self):
        """Initialize topic subscriptions after ROS node is ready"""
        # Refresh available topics
        self._on_refresh_topics()
        
        # Subscribe to default topics
        front_topic = self.front_image_panel.get_current_topic()
        if front_topic:
            self.ros_node.subscribe_front_image(front_topic)
        
        rear_topic = self.rear_image_panel.get_current_topic()
        if rear_topic:
            self.ros_node.subscribe_rear_image(rear_topic)
    
    def _connect_signals(self):
        """Connect all signals"""
        # Motor panel signals
        self.front_motor_panel.enable_btn.clicked.connect(self._on_front_motor_enable)
        self.front_motor_panel.disable_btn.clicked.connect(self._on_front_motor_disable)
        self.front_motor_panel.set_speed_btn.clicked.connect(self._on_front_motor_set_speed)
        
        self.rear_motor_panel.enable_btn.clicked.connect(self._on_rear_motor_enable)
        self.rear_motor_panel.disable_btn.clicked.connect(self._on_rear_motor_disable)
        self.rear_motor_panel.set_speed_btn.clicked.connect(self._on_rear_motor_set_speed)
        
        # Sync controls
        self.sync_enable_btn.clicked.connect(self._on_sync_enable)
        self.sync_disable_btn.clicked.connect(self._on_sync_disable)
        self.sync_set_btn.clicked.connect(self._on_sync_set_speed)
        self.emergency_stop_btn.clicked.connect(self._on_emergency_stop)
        
        # Light panel signals
        self.light1_panel.toggle_btn.clicked.connect(self._on_light1_toggle)
        self.light1_panel.brightness_slider.valueChanged.connect(self._on_light1_brightness)
        self.light1_panel.brightness_spinbox.valueChanged.connect(self._on_light1_brightness)
        
        self.light2_panel.toggle_btn.clicked.connect(self._on_light2_toggle)
        self.light2_panel.brightness_slider.valueChanged.connect(self._on_light2_brightness)
        self.light2_panel.brightness_spinbox.valueChanged.connect(self._on_light2_brightness)
        
        # Image panel signals
        self.front_image_panel.reset_btn.clicked.connect(self._on_front_reset)
        self.rear_image_panel.reset_btn.clicked.connect(self._on_rear_reset)
        self.front_image_panel.topic_changed.connect(self._on_front_topic_changed)
        self.rear_image_panel.topic_changed.connect(self._on_rear_topic_changed)
        self.front_image_panel.refresh_topics_requested.connect(self._on_refresh_topics)
        self.rear_image_panel.refresh_topics_requested.connect(self._on_refresh_topics)
        
        # ROS signal bridge connections
        self.signal_bridge.front_velocity_updated.connect(self.front_motor_panel.update_velocity)
        self.signal_bridge.rear_velocity_updated.connect(self.rear_motor_panel.update_velocity)
        
        self.signal_bridge.light1_status_updated.connect(self.light1_panel.update_status)
        self.signal_bridge.light2_status_updated.connect(self.light2_panel.update_status)
        self.signal_bridge.light1_brightness_updated.connect(self.light1_panel.update_brightness)
        self.signal_bridge.light2_brightness_updated.connect(self.light2_panel.update_brightness)
        self.signal_bridge.light1_voltage_updated.connect(self.light1_panel.update_voltage)
        self.signal_bridge.light1_current_updated.connect(self.light1_panel.update_current)
        self.signal_bridge.light2_voltage_updated.connect(self.light2_panel.update_voltage)
        self.signal_bridge.light2_current_updated.connect(self.light2_panel.update_current)
        self.signal_bridge.connection_status_updated.connect(self._on_connection_status)
        
        self.signal_bridge.front_image_updated.connect(self.front_image_panel.update_image)
        self.signal_bridge.rear_image_updated.connect(self.rear_image_panel.update_image)
    
    # Motor control handlers
    def _on_front_motor_enable(self):
        self.ros_node.publish_front_enable(True)
        self._front_motor_enabled = True
        self.front_motor_panel.set_enabled_state(True)
        self.status_label.setText("Front motor enabled")
    
    def _on_front_motor_disable(self):
        self.ros_node.publish_front_speed(0)
        self.ros_node.publish_front_enable(False)
        self._front_motor_enabled = False
        self.front_motor_panel.set_enabled_state(False)
        self.front_motor_panel.speed_slider.setValue(0)
        self.status_label.setText("Front motor disabled")
    
    def _on_front_motor_set_speed(self):
        speed = self.front_motor_panel.get_speed()
        self.ros_node.publish_front_speed(speed)
        self.status_label.setText(f"Front motor speed set to: {speed:.1f} RPM")
    
    def _on_rear_motor_enable(self):
        self.ros_node.publish_rear_enable(True)
        self._rear_motor_enabled = True
        self.rear_motor_panel.set_enabled_state(True)
        self.status_label.setText("Rear motor enabled")
    
    def _on_rear_motor_disable(self):
        self.ros_node.publish_rear_speed(0)
        self.ros_node.publish_rear_enable(False)
        self._rear_motor_enabled = False
        self.rear_motor_panel.set_enabled_state(False)
        self.rear_motor_panel.speed_slider.setValue(0)
        self.status_label.setText("Rear motor disabled")
    
    def _on_rear_motor_set_speed(self):
        speed = self.rear_motor_panel.get_speed()
        self.ros_node.publish_rear_speed(speed)
        self.status_label.setText(f"Rear motor speed set to: {speed:.1f} RPM")
    
    def _on_sync_enable(self):
        self._on_front_motor_enable()
        self._on_rear_motor_enable()
        self.status_label.setText("Both motors enabled")
    
    def _on_sync_disable(self):
        self._on_front_motor_disable()
        self._on_rear_motor_disable()
        self.status_label.setText("Both motors disabled")
    
    def _on_sync_set_speed(self):
        speed = self.sync_speed_spinbox.value()
        self.front_motor_panel.speed_slider.setValue(int(speed))
        self.rear_motor_panel.speed_slider.setValue(int(speed))
        self.ros_node.publish_front_speed(speed)
        self.ros_node.publish_rear_speed(speed)
        self.status_label.setText(f"Both motors speed set to: {speed:.1f} RPM")
    
    def _on_emergency_stop(self):
        self.ros_node.publish_front_speed(0)
        self.ros_node.publish_rear_speed(0)
        self.ros_node.publish_front_enable(False)
        self.ros_node.publish_rear_enable(False)
        
        self._front_motor_enabled = False
        self._rear_motor_enabled = False
        self.front_motor_panel.set_enabled_state(False)
        self.rear_motor_panel.set_enabled_state(False)
        self.front_motor_panel.speed_slider.setValue(0)
        self.rear_motor_panel.speed_slider.setValue(0)
        
        self.status_label.setText("EMERGENCY STOP - All motors stopped")
    
    # Light control handlers
    def _on_light1_toggle(self):
        self._light1_enabled = not self._light1_enabled
        self.ros_node.publish_light1_control(self._light1_enabled)
        self.light1_panel.update_status(self._light1_enabled)
        status = "ON" if self._light1_enabled else "OFF"
        self.status_label.setText(f"Light 1: {status}")
    
    def _on_light2_toggle(self):
        self._light2_enabled = not self._light2_enabled
        self.ros_node.publish_light2_control(self._light2_enabled)
        self.light2_panel.update_status(self._light2_enabled)
        status = "ON" if self._light2_enabled else "OFF"
        self.status_label.setText(f"Light 2: {status}")
    
    def _on_light1_brightness(self, value):
        self.ros_node.publish_light1_brightness(value)
    
    def _on_light2_brightness(self, value):
        self.ros_node.publish_light2_brightness(value)
    
    def _on_connection_status(self, connected: bool):
        if connected:
            self.light_connection_label.setText("Light Controller: Connected")
            self.light_connection_label.setStyleSheet("color: #008000; font-weight: bold;")
        else:
            self.light_connection_label.setText("Light Controller: Disconnected")
            self.light_connection_label.setStyleSheet("color: #aa0000; font-weight: bold;")
    
    # Topic selection handlers
    def _on_front_topic_changed(self, topic: str):
        """Handle front image topic change"""
        self.ros_node.subscribe_front_image(topic)
        self.front_image_panel.clear_image()
        self.status_label.setText(f"Subscribed to: {topic}")
    
    def _on_rear_topic_changed(self, topic: str):
        """Handle rear image topic change"""
        self.ros_node.subscribe_rear_image(topic)
        self.rear_image_panel.clear_image()
        self.status_label.setText(f"Subscribed to: {topic}")
    
    def _on_refresh_topics(self):
        """Refresh available image topics"""
        topics = self.ros_node.get_available_image_topics()
        self.front_image_panel.update_topic_list(topics)
        self.rear_image_panel.update_topic_list(topics)
        self.status_label.setText(f"Found {len(topics)} image topics")
    
    # Image reset handlers
    def _on_front_reset(self):
        self.front_image_panel.clear_image()
        
        def callback(future):
            try:
                result = future.result()
                if result.success:
                    self.status_label.setText("Front camera stitching reset")
                else:
                    self.status_label.setText(f"Reset failed: {result.message}")
            except Exception as e:
                self.status_label.setText(f"Reset error: {e}")
        
        if not self.ros_node.reset_front_stitching(callback):
            self.status_label.setText("Front camera reset service unavailable")
    
    def _on_rear_reset(self):
        self.rear_image_panel.clear_image()
        
        def callback(future):
            try:
                result = future.result()
                if result.success:
                    self.status_label.setText("Rear camera stitching reset")
                else:
                    self.status_label.setText(f"Reset failed: {result.message}")
            except Exception as e:
                self.status_label.setText(f"Reset error: {e}")
        
        if not self.ros_node.reset_rear_stitching(callback):
            self.status_label.setText("Rear camera reset service unavailable")
    
    def closeEvent(self, event):
        """Handle window close - emergency stop all"""
        self._on_emergency_stop()
        event.accept()


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
    ros_node = WorkflowNode(signal_bridge)
    
    # Create and show GUI
    gui = WorkflowGUI(ros_node, signal_bridge)
    gui.showFullScreen()
    
    # Create executor for ROS2
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    # Run ROS2 spin in separate thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    # Initialize topics after ROS thread is running
    QTimer.singleShot(500, gui.init_topics)
    
    # Run Qt event loop
    exit_code = app.exec_()
    
    # Cleanup
    ros_node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
