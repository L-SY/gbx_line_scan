#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Print Workflow GUI

GUI for controlling a single XYT300 motor.
No popup dialogs for connection/initialization - silent operation.

Topics:
    Motor Control:
        motor1/motor/speed_cmd (Int16): Target speed for motor 1 (RPM)
        motor1/motor/control_cmd (Int16): Control command for motor 1 (1=forward, 2=reverse, 5=stop)
    
    Motor Status:
        motor1/motor/speed (Int16): Current speed of motor 1 (RPM)
"""

import sys
import threading
from typing import Optional

# PyQt5 imports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QGroupBox, QFrame,
    QDoubleSpinBox, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QFont
import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int16


# ============================================================================
# ROS Signal Bridge for Thread-Safe Qt Updates
# ============================================================================
class RosSignalBridge(QObject):
    """Bridge to safely emit Qt signals from ROS callbacks"""
    # Motor signals
    motor1_speed_updated = pyqtSignal(int)


# ============================================================================
# ROS2 Node
# ============================================================================
class PrintWorkflowNode(Node):
    """ROS2 Node for print workflow GUI"""
    
    def __init__(self, signal_bridge: RosSignalBridge):
        super().__init__('print_workflow')
        self.signal_bridge = signal_bridge
        
        self._setup_motor_interfaces()
        
        self.get_logger().info('Print Workflow GUI Node initialized')
    
    def _setup_motor_interfaces(self):
        """Setup motor control publishers and subscribers"""
        # Publishers for speed commands (only motor1)
        self.motor1_speed_pub = self.create_publisher(
            Int16, 'motor1/motor/speed_cmd', 10)
        
        # Publishers for control commands (only motor1)
        self.motor1_control_pub = self.create_publisher(
            Int16, 'motor1/motor/control_cmd', 10)
        
        # Subscribers for current speed (only motor1)
        self.motor1_speed_sub = self.create_subscription(
            Int16, 'motor1/motor/speed',
            self._motor1_speed_callback, 10)
        
        self.get_logger().info('Subscribed to motor speed topic:')
        self.get_logger().info('  - motor1/motor/speed')
        
        # Add a timer to check if we're receiving speed updates
        self._speed_check_timer = QTimer()
        self._speed_check_timer.timeout.connect(self._check_speed_reception)
        self._speed_check_timer.start(5000)  # Check every 5 seconds
        self._last_motor1_speed_time = None
    
    # Motor callbacks
    def _motor1_speed_callback(self, msg: Int16):
        import time
        self._last_motor1_speed_time = time.time()
        self.get_logger().debug(f'Motor1 speed received: {msg.data} RPM')
        self.signal_bridge.motor1_speed_updated.emit(msg.data)
    
    def _check_speed_reception(self):
        """Check if we're receiving speed updates from motor"""
        import time
        current_time = time.time()
        timeout = 6.0  # 6 seconds timeout
        
        if self._last_motor1_speed_time is None or (current_time - self._last_motor1_speed_time) > timeout:
            self.get_logger().warn('Not receiving speed updates from motor1')
    
    # Motor control methods
    def publish_motor1_speed(self, speed: int):
        """Publish speed command for motor 1 (0-85 RPM)"""
        msg = Int16()
        msg.data = max(0, min(85, int(speed)))  # Clamp to 0-85
        self.motor1_speed_pub.publish(msg)
    
    def publish_motor1_control(self, command: int):
        """Publish control command for motor 1 (1=forward, 2=reverse, 5=stop)"""
        msg = Int16()
        msg.data = int(command)
        self.motor1_control_pub.publish(msg)


# ============================================================================
# Motor Control Panel Widget
# ============================================================================
class MotorControlPanel(QGroupBox):
    """Control panel for a single motor"""
    
    def __init__(self, motor_name: str, motor_id: int, parent=None):
        super().__init__(motor_name, parent)
        self.motor_name = motor_name
        self.motor_id = motor_id
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout()
        layout.setSpacing(8)
        
        # Status indicator
        status_layout = QHBoxLayout()
        self.status_label = QLabel("状态: 就绪")
        self.status_label.setStyleSheet("color: #808080; font-weight: bold;")
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        layout.addLayout(status_layout)
        
        # Speed control section
        speed_label = QLabel("速度控制 (RPM):")
        speed_label.setFont(QFont("Arial", 9, QFont.Bold))
        layout.addWidget(speed_label)
        
        # Speed slider
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(85)
        self.speed_slider.setValue(0)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(17)  # 85/5 = 17
        layout.addWidget(self.speed_slider)
        
        # Speed input and buttons
        input_layout = QHBoxLayout()
        
        self.speed_spinbox = QDoubleSpinBox()
        self.speed_spinbox.setMinimum(0)
        self.speed_spinbox.setMaximum(85)
        self.speed_spinbox.setValue(0)
        self.speed_spinbox.setSuffix(" RPM")
        self.speed_spinbox.setDecimals(0)
        self.speed_spinbox.setSingleStep(1)
        
        self.set_speed_btn = QPushButton("设置")
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
        
        self.stop_btn = QPushButton("停止")
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
        
        # Direction control buttons
        dir_layout = QHBoxLayout()
        self.forward_btn = QPushButton("正转")
        self.forward_btn.setStyleSheet("""
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
        
        self.reverse_btn = QPushButton("反转")
        self.reverse_btn.setStyleSheet("""
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
        
        dir_layout.addWidget(self.forward_btn)
        dir_layout.addWidget(self.reverse_btn)
        layout.addLayout(dir_layout)
        
        # Separator
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setStyleSheet("background-color: #c0c0c0;")
        layout.addWidget(line)
        
        # Current velocity display
        vel_layout = QHBoxLayout()
        vel_label = QLabel("当前速度:")
        self.velocity_display = QLabel("-- RPM")
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
        
        # Track if we've received any speed updates
        self._speed_received = False
        
        self.setLayout(layout)
        
        # Connect internal signals
        self.speed_slider.valueChanged.connect(self._on_slider_changed)
        self.speed_spinbox.valueChanged.connect(self._on_spinbox_changed)
        self.stop_btn.clicked.connect(self._on_stop_clicked)
        
        # Store reference to parent GUI for speed updates
        self.parent_gui = None
    
    def set_parent_gui(self, parent_gui):
        """Set reference to parent GUI for speed updates"""
        self.parent_gui = parent_gui
    
    def _on_slider_changed(self, value):
        self.speed_spinbox.blockSignals(True)
        self.speed_spinbox.setValue(value)
        self.speed_spinbox.blockSignals(False)
        # Automatically set speed when slider changes
        if self.parent_gui:
            self.parent_gui._on_motor1_set_speed()
    
    def _on_spinbox_changed(self, value):
        self.speed_slider.blockSignals(True)
        self.speed_slider.setValue(int(value))
        self.speed_slider.blockSignals(False)
        # Automatically set speed when spinbox changes
        if self.parent_gui:
            self.parent_gui._on_motor1_set_speed()
    
    def _on_stop_clicked(self):
        # Stop sets speed to 10
        self.speed_slider.setValue(10)
        self.speed_spinbox.setValue(10)
        if self.parent_gui:
            self.parent_gui._on_motor1_stop()
    
    def update_velocity(self, velocity: int):
        """Update current velocity display"""
        self._speed_received = True
        self.velocity_display.setText(f"{velocity} RPM")
    
    def get_speed(self) -> int:
        """Get current speed setting"""
        return int(self.speed_spinbox.value())


# ============================================================================
# Main Print Workflow GUI Window
# ============================================================================
class PrintWorkflowGUI(QMainWindow):
    """Main print workflow GUI window"""
    
    def __init__(self, ros_node: PrintWorkflowNode, signal_bridge: RosSignalBridge):
        super().__init__()
        self.ros_node = ros_node
        self.signal_bridge = signal_bridge
        
        self._setup_ui()
        self._connect_signals()
    
    def _setup_ui(self):
        self.setWindowTitle("打印工作流控制 - 单电机控制")
        self.setMinimumSize(600, 500)
        
        # Apply light gray/white theme (similar to workflow_gui)
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
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # Title
        title = QLabel("打印工作流控制 - 单电机控制")
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
        
        # Motor control section
        motor_group = QGroupBox("电机控制")
        motor_layout = QVBoxLayout(motor_group)
        
        # Single motor panel
        self.motor1_panel = MotorControlPanel("电机 1 (ID: 1)", 1)
        motor_layout.addWidget(self.motor1_panel)
        
        # Emergency stop button
        emergency_layout = QHBoxLayout()
        emergency_layout.addStretch()
        self.emergency_stop_btn = QPushButton("紧急停止")
        self.emergency_stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #aa0000;
                color: #ffffff;
                border: 1px solid #880000;
                padding: 8px 20px;
                border-radius: 4px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover { background-color: #bb0000; }
        """)
        emergency_layout.addWidget(self.emergency_stop_btn)
        emergency_layout.addStretch()
        
        motor_layout.addLayout(emergency_layout)
        main_layout.addWidget(motor_group)
        
        # Status bar
        status_layout = QHBoxLayout()
        self.status_label = QLabel("就绪")
        self.status_label.setStyleSheet("background-color: #f0f0f0; color: #606060; font-size: 10px; padding: 3px;")
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        
        main_layout.addLayout(status_layout)
        
        central_widget.setLayout(main_layout)
    
    def _connect_signals(self):
        """Connect all signals"""
        # Set parent GUI reference in motor panel for automatic speed updates
        self.motor1_panel.set_parent_gui(self)
        
        # Motor panel signals
        self.motor1_panel.set_speed_btn.clicked.connect(self._on_motor1_set_speed)
        self.motor1_panel.stop_btn.clicked.connect(self._on_motor1_stop)
        self.motor1_panel.forward_btn.clicked.connect(self._on_motor1_forward)
        self.motor1_panel.reverse_btn.clicked.connect(self._on_motor1_reverse)
        
        # Emergency stop
        self.emergency_stop_btn.clicked.connect(self._on_emergency_stop)
        
        # ROS signal bridge connections
        self.signal_bridge.motor1_speed_updated.connect(self.motor1_panel.update_velocity)
    
    # Motor control handlers
    def _on_motor1_set_speed(self):
        speed = self.motor1_panel.get_speed()
        # If speed > 0, set forward direction and speed; if 0, stop
        if speed > 0:
            self.ros_node.publish_motor1_control(1)  # FORWARD
            # Delay speed command to ensure direction is set first
            QTimer.singleShot(50, lambda: self.ros_node.publish_motor1_speed(speed))
        else:
            self.ros_node.publish_motor1_control(5)  # STOP
            self.ros_node.publish_motor1_speed(0)
        self.status_label.setText(f"电机1速度设置为: {speed} RPM")
    
    def _on_motor1_stop(self):
        # Stop by setting speed to 10 (as requested)
        self.ros_node.publish_motor1_speed(10)
        self.status_label.setText("电机1已停止 (速度设为10 RPM)")
    
    def _on_motor1_forward(self):
        # Like xyt300_motor_gui: only send direction command, don't send speed
        # The motor will use the previously set speed
        self.ros_node.publish_motor1_control(1)  # FORWARD
        self.status_label.setText("电机1正转")
    
    def _on_motor1_reverse(self):
        # Like xyt300_motor_gui: only send direction command, don't send speed
        # The motor will use the previously set speed
        self.ros_node.publish_motor1_control(2)  # REVERSE
        self.status_label.setText("电机1反转")
    
    def _on_emergency_stop(self):
        self._on_motor1_stop()
        self.status_label.setText("紧急停止 - 电机已停止")
    
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
    ros_node = PrintWorkflowNode(signal_bridge)
    
    # Create and show GUI
    gui = PrintWorkflowGUI(ros_node, signal_bridge)
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
