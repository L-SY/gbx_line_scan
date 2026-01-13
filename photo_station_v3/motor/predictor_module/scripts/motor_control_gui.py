#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motor Control GUI for Predictor Module

This GUI provides a visual interface to control dual LC servo motors via ROS2 topics.

Topics Published:
    /predictor/front_motor/cmd_vel (Float64): Target speed for front motor (RPM)
    /predictor/rear_motor/cmd_vel (Float64): Target speed for rear motor (RPM)
    /predictor/front_motor/enable (Bool): Enable/disable front motor
    /predictor/rear_motor/enable (Bool): Enable/disable rear motor

Topics Subscribed:
    /predictor/front_motor/velocity (Float64): Current speed of front motor (RPM)
    /predictor/rear_motor/velocity (Float64): Current speed of rear motor (RPM)
"""

import sys
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64, Bool

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QSpinBox, QPushButton, QGroupBox, QFrame,
    QDoubleSpinBox, QGridLayout, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QPalette, QColor


class RosSignalBridge(QObject):
    """Bridge to safely emit Qt signals from ROS callbacks"""
    front_velocity_updated = pyqtSignal(float)
    rear_velocity_updated = pyqtSignal(float)


class MotorControlNode(Node):
    """ROS2 Node for motor control GUI"""
    
    def __init__(self, signal_bridge: RosSignalBridge):
        super().__init__('motor_control_gui')
        self.signal_bridge = signal_bridge
        
        # Publishers for motor control
        self.front_cmd_pub = self.create_publisher(
            Float64, '/predictor/front_motor/cmd_vel', 10)
        self.rear_cmd_pub = self.create_publisher(
            Float64, '/predictor/rear_motor/cmd_vel', 10)
        self.front_enable_pub = self.create_publisher(
            Bool, '/predictor/front_motor/enable', 10)
        self.rear_enable_pub = self.create_publisher(
            Bool, '/predictor/rear_motor/enable', 10)
        
        # Subscribers for velocity feedback
        self.front_vel_sub = self.create_subscription(
            Float64, '/predictor/front_motor/velocity', 
            self.front_velocity_callback, 10)
        self.rear_vel_sub = self.create_subscription(
            Float64, '/predictor/rear_motor/velocity',
            self.rear_velocity_callback, 10)
        
        self.get_logger().info('Motor Control GUI Node initialized')
    
    def front_velocity_callback(self, msg: Float64):
        """Callback for front motor velocity"""
        self.signal_bridge.front_velocity_updated.emit(msg.data)
    
    def rear_velocity_callback(self, msg: Float64):
        """Callback for rear motor velocity"""
        self.signal_bridge.rear_velocity_updated.emit(msg.data)
    
    def publish_front_speed(self, speed: float):
        """Publish front motor speed command"""
        msg = Float64()
        msg.data = speed
        self.front_cmd_pub.publish(msg)
        self.get_logger().debug(f'Front motor speed: {speed:.2f} RPM')
    
    def publish_rear_speed(self, speed: float):
        """Publish rear motor speed command"""
        msg = Float64()
        msg.data = speed
        self.rear_cmd_pub.publish(msg)
        self.get_logger().debug(f'Rear motor speed: {speed:.2f} RPM')
    
    def publish_front_enable(self, enable: bool):
        """Publish front motor enable command"""
        msg = Bool()
        msg.data = enable
        self.front_enable_pub.publish(msg)
        self.get_logger().info(f'Front motor {"enabled" if enable else "disabled"}')
    
    def publish_rear_enable(self, enable: bool):
        """Publish rear motor enable command"""
        msg = Bool()
        msg.data = enable
        self.rear_enable_pub.publish(msg)
        self.get_logger().info(f'Rear motor {"enabled" if enable else "disabled"}')


class MotorControlPanel(QGroupBox):
    """Control panel for a single motor"""
    
    def __init__(self, motor_name: str, parent=None):
        super().__init__(motor_name, parent)
        self.motor_name = motor_name
        self._enabled = False
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout()
        layout.setSpacing(10)
        
        # Status indicator
        status_layout = QHBoxLayout()
        self.status_label = QLabel("状态: 未启用")
        self.status_label.setStyleSheet("color: #888888; font-weight: bold;")
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        layout.addLayout(status_layout)
        
        # Enable/Disable buttons
        btn_layout = QHBoxLayout()
        self.enable_btn = QPushButton("启用电机")
        self.enable_btn.setStyleSheet("""
            QPushButton {
                background-color: #ffffff;
                color: #000000;
                border: 2px solid #000000;
                padding: 10px 20px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
            QPushButton:pressed {
                background-color: #c0c0c0;
            }
        """)
        
        self.disable_btn = QPushButton("禁用电机")
        self.disable_btn.setStyleSheet("""
            QPushButton {
                background-color: #333333;
                color: #ffffff;
                border: 2px solid #333333;
                padding: 10px 20px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #444444;
            }
            QPushButton:pressed {
                background-color: #222222;
            }
        """)
        
        btn_layout.addWidget(self.enable_btn)
        btn_layout.addWidget(self.disable_btn)
        layout.addLayout(btn_layout)
        
        # Separator
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)
        
        # Speed control section
        speed_label = QLabel("速度控制 (RPM):")
        speed_label.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(speed_label)
        
        # Speed slider
        slider_layout = QHBoxLayout()
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(-3000)
        self.speed_slider.setMaximum(3000)
        self.speed_slider.setValue(0)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(500)
        slider_layout.addWidget(self.speed_slider)
        layout.addLayout(slider_layout)
        
        # Speed input and set button
        input_layout = QHBoxLayout()
        
        self.speed_spinbox = QDoubleSpinBox()
        self.speed_spinbox.setMinimum(-3000)
        self.speed_spinbox.setMaximum(3000)
        self.speed_spinbox.setValue(0)
        self.speed_spinbox.setSuffix(" RPM")
        self.speed_spinbox.setDecimals(1)
        self.speed_spinbox.setSingleStep(10)
        
        self.set_speed_btn = QPushButton("设置速度")
        self.set_speed_btn.setStyleSheet("""
            QPushButton {
                background-color: #666666;
                color: #ffffff;
                border: none;
                padding: 8px 15px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #777777;
            }
            QPushButton:pressed {
                background-color: #555555;
            }
        """)
        
        self.stop_btn = QPushButton("停止")
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #999999;
                color: #000000;
                border: none;
                padding: 8px 15px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #aaaaaa;
            }
            QPushButton:pressed {
                background-color: #888888;
            }
        """)
        
        input_layout.addWidget(self.speed_spinbox)
        input_layout.addWidget(self.set_speed_btn)
        input_layout.addWidget(self.stop_btn)
        layout.addLayout(input_layout)
        
        # Separator
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line2)
        
        # Current velocity display
        vel_layout = QHBoxLayout()
        vel_label = QLabel("当前速度:")
        vel_label.setFont(QFont("Arial", 10, QFont.Bold))
        self.velocity_display = QLabel("0.00 RPM")
        self.velocity_display.setStyleSheet("""
            QLabel {
                background-color: #000000;
                color: #ffffff;
                padding: 10px 20px;
                border-radius: 5px;
                font-size: 18px;
                font-weight: bold;
                font-family: 'Courier New', monospace;
            }
        """)
        self.velocity_display.setAlignment(Qt.AlignCenter)
        vel_layout.addWidget(vel_label)
        vel_layout.addWidget(self.velocity_display)
        layout.addLayout(vel_layout)
        
        self.setLayout(layout)
        
        # Connect signals
        self.speed_slider.valueChanged.connect(self._on_slider_changed)
        self.speed_spinbox.valueChanged.connect(self._on_spinbox_changed)
        self.stop_btn.clicked.connect(self._on_stop_clicked)
    
    def _on_slider_changed(self, value):
        """Sync spinbox with slider"""
        self.speed_spinbox.blockSignals(True)
        self.speed_spinbox.setValue(value)
        self.speed_spinbox.blockSignals(False)
    
    def _on_spinbox_changed(self, value):
        """Sync slider with spinbox"""
        self.speed_slider.blockSignals(True)
        self.speed_slider.setValue(int(value))
        self.speed_slider.blockSignals(False)
    
    def _on_stop_clicked(self):
        """Stop button clicked"""
        self.speed_slider.setValue(0)
        self.speed_spinbox.setValue(0)
    
    def update_velocity(self, velocity: float):
        """Update the velocity display"""
        self.velocity_display.setText(f"{velocity:.2f} RPM")
    
    def set_enabled_state(self, enabled: bool):
        """Update the enabled state UI"""
        self._enabled = enabled
        if enabled:
            self.status_label.setText("状态: 已启用")
            self.status_label.setStyleSheet("color: #ffffff; font-weight: bold;")
        else:
            self.status_label.setText("状态: 未启用")
            self.status_label.setStyleSheet("color: #888888; font-weight: bold;")
    
    def get_speed(self) -> float:
        """Get the current speed setting"""
        return self.speed_spinbox.value()


class MotorControlGUI(QMainWindow):
    """Main window for motor control GUI"""
    
    def __init__(self, ros_node: MotorControlNode, signal_bridge: RosSignalBridge):
        super().__init__()
        self.ros_node = ros_node
        self.signal_bridge = signal_bridge
        
        self._front_enabled = False
        self._rear_enabled = False
        
        self._setup_ui()
        self._connect_signals()
    
    def _setup_ui(self):
        self.setWindowTitle("电机控制面板 - Predictor Module")
        self.setMinimumSize(800, 600)
        
        # Set black/white/gray theme
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1a1a1a;
            }
            QGroupBox {
                background-color: #2a2a2a;
                border: 2px solid #444444;
                border-radius: 10px;
                margin-top: 15px;
                padding: 15px;
                font-size: 14px;
                font-weight: bold;
                color: #ffffff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 20px;
                padding: 0 10px;
            }
            QLabel {
                color: #e0e0e0;
            }
            QSlider::groove:horizontal {
                border: 1px solid #555555;
                height: 8px;
                background: #333333;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #ffffff;
                border: 1px solid #cccccc;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QSlider::handle:horizontal:hover {
                background: #e0e0e0;
            }
            QDoubleSpinBox, QSpinBox {
                background-color: #333333;
                color: #ffffff;
                border: 1px solid #666666;
                border-radius: 5px;
                padding: 5px;
                font-size: 12px;
            }
            QDoubleSpinBox:focus, QSpinBox:focus {
                border: 2px solid #888888;
            }
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout()
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # Title
        title = QLabel("双电机控制系统")
        title.setStyleSheet("""
            QLabel {
                font-size: 24px;
                font-weight: bold;
                color: #ffffff;
                padding: 10px;
            }
        """)
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        # Motor panels
        motors_layout = QHBoxLayout()
        motors_layout.setSpacing(20)
        
        self.front_panel = MotorControlPanel("前电机 (Front Motor)")
        self.rear_panel = MotorControlPanel("后电机 (Rear Motor)")
        
        motors_layout.addWidget(self.front_panel)
        motors_layout.addWidget(self.rear_panel)
        
        main_layout.addLayout(motors_layout)
        
        # Emergency stop button
        emergency_layout = QHBoxLayout()
        emergency_layout.addStretch()
        
        self.emergency_stop_btn = QPushButton("紧急停止所有电机")
        self.emergency_stop_btn.setMinimumSize(300, 60)
        self.emergency_stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #000000;
                color: #ffffff;
                border: 3px solid #ffffff;
                padding: 15px 30px;
                border-radius: 10px;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #333333;
            }
            QPushButton:pressed {
                background-color: #555555;
            }
        """)
        
        emergency_layout.addWidget(self.emergency_stop_btn)
        emergency_layout.addStretch()
        main_layout.addLayout(emergency_layout)
        
        # Sync control section
        sync_group = QGroupBox("同步控制")
        sync_layout = QHBoxLayout()
        
        self.sync_speed_spinbox = QDoubleSpinBox()
        self.sync_speed_spinbox.setMinimum(-3000)
        self.sync_speed_spinbox.setMaximum(3000)
        self.sync_speed_spinbox.setValue(0)
        self.sync_speed_spinbox.setSuffix(" RPM")
        self.sync_speed_spinbox.setDecimals(1)
        self.sync_speed_spinbox.setSingleStep(10)
        
        self.sync_enable_btn = QPushButton("同时启用两电机")
        self.sync_enable_btn.setStyleSheet("""
            QPushButton {
                background-color: #ffffff;
                color: #000000;
                border: 2px solid #000000;
                padding: 10px 20px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
        """)
        
        self.sync_disable_btn = QPushButton("同时禁用两电机")
        self.sync_disable_btn.setStyleSheet("""
            QPushButton {
                background-color: #333333;
                color: #ffffff;
                border: 2px solid #333333;
                padding: 10px 20px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #444444;
            }
        """)
        
        self.sync_set_speed_btn = QPushButton("同时设置速度")
        self.sync_set_speed_btn.setStyleSheet("""
            QPushButton {
                background-color: #666666;
                color: #ffffff;
                border: none;
                padding: 10px 20px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #777777;
            }
        """)
        
        sync_layout.addWidget(QLabel("同步速度:"))
        sync_layout.addWidget(self.sync_speed_spinbox)
        sync_layout.addWidget(self.sync_set_speed_btn)
        sync_layout.addWidget(self.sync_enable_btn)
        sync_layout.addWidget(self.sync_disable_btn)
        sync_layout.addStretch()
        
        sync_group.setLayout(sync_layout)
        main_layout.addWidget(sync_group)
        
        # Status bar info
        status_label = QLabel("ROS2 Topics: /predictor/[front|rear]_motor/[cmd_vel|enable|velocity]")
        status_label.setStyleSheet("color: #888888; font-size: 10px;")
        status_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(status_label)
        
        central_widget.setLayout(main_layout)
    
    def _connect_signals(self):
        """Connect all signals to slots"""
        # Front motor signals
        self.front_panel.enable_btn.clicked.connect(self._on_front_enable)
        self.front_panel.disable_btn.clicked.connect(self._on_front_disable)
        self.front_panel.set_speed_btn.clicked.connect(self._on_front_set_speed)
        
        # Rear motor signals
        self.rear_panel.enable_btn.clicked.connect(self._on_rear_enable)
        self.rear_panel.disable_btn.clicked.connect(self._on_rear_disable)
        self.rear_panel.set_speed_btn.clicked.connect(self._on_rear_set_speed)
        
        # Emergency stop
        self.emergency_stop_btn.clicked.connect(self._on_emergency_stop)
        
        # Sync controls
        self.sync_enable_btn.clicked.connect(self._on_sync_enable)
        self.sync_disable_btn.clicked.connect(self._on_sync_disable)
        self.sync_set_speed_btn.clicked.connect(self._on_sync_set_speed)
        
        # ROS velocity updates
        self.signal_bridge.front_velocity_updated.connect(self.front_panel.update_velocity)
        self.signal_bridge.rear_velocity_updated.connect(self.rear_panel.update_velocity)
    
    def _on_front_enable(self):
        """Enable front motor"""
        self.ros_node.publish_front_enable(True)
        self._front_enabled = True
        self.front_panel.set_enabled_state(True)
    
    def _on_front_disable(self):
        """Disable front motor"""
        self.ros_node.publish_front_speed(0)
        self.ros_node.publish_front_enable(False)
        self._front_enabled = False
        self.front_panel.set_enabled_state(False)
        self.front_panel.speed_slider.setValue(0)
        self.front_panel.speed_spinbox.setValue(0)
    
    def _on_front_set_speed(self):
        """Set front motor speed"""
        speed = self.front_panel.get_speed()
        self.ros_node.publish_front_speed(speed)
    
    def _on_rear_enable(self):
        """Enable rear motor"""
        self.ros_node.publish_rear_enable(True)
        self._rear_enabled = True
        self.rear_panel.set_enabled_state(True)
    
    def _on_rear_disable(self):
        """Disable rear motor"""
        self.ros_node.publish_rear_speed(0)
        self.ros_node.publish_rear_enable(False)
        self._rear_enabled = False
        self.rear_panel.set_enabled_state(False)
        self.rear_panel.speed_slider.setValue(0)
        self.rear_panel.speed_spinbox.setValue(0)
    
    def _on_rear_set_speed(self):
        """Set rear motor speed"""
        speed = self.rear_panel.get_speed()
        self.ros_node.publish_rear_speed(speed)
    
    def _on_emergency_stop(self):
        """Emergency stop all motors"""
        # Stop and disable all motors
        self.ros_node.publish_front_speed(0)
        self.ros_node.publish_rear_speed(0)
        self.ros_node.publish_front_enable(False)
        self.ros_node.publish_rear_enable(False)
        
        # Update UI
        self._front_enabled = False
        self._rear_enabled = False
        self.front_panel.set_enabled_state(False)
        self.rear_panel.set_enabled_state(False)
        self.front_panel.speed_slider.setValue(0)
        self.front_panel.speed_spinbox.setValue(0)
        self.rear_panel.speed_slider.setValue(0)
        self.rear_panel.speed_spinbox.setValue(0)
    
    def _on_sync_enable(self):
        """Enable both motors simultaneously"""
        self._on_front_enable()
        self._on_rear_enable()
    
    def _on_sync_disable(self):
        """Disable both motors simultaneously"""
        self._on_front_disable()
        self._on_rear_disable()
    
    def _on_sync_set_speed(self):
        """Set both motors to same speed"""
        speed = self.sync_speed_spinbox.value()
        self.front_panel.speed_slider.setValue(int(speed))
        self.front_panel.speed_spinbox.setValue(speed)
        self.rear_panel.speed_slider.setValue(int(speed))
        self.rear_panel.speed_spinbox.setValue(speed)
        self.ros_node.publish_front_speed(speed)
        self.ros_node.publish_rear_speed(speed)
    
    def closeEvent(self, event):
        """Handle window close event"""
        # Stop all motors before closing
        self._on_emergency_stop()
        event.accept()


def main():
    """Main entry point"""
    # Initialize ROS2
    rclpy.init()
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Create signal bridge for ROS-Qt communication
    signal_bridge = RosSignalBridge()
    
    # Create ROS2 node
    ros_node = MotorControlNode(signal_bridge)
    
    # Create and show GUI
    gui = MotorControlGUI(ros_node, signal_bridge)
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
