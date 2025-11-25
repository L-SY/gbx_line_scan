#!/usr/bin/env python3
"""
电机测试脚本
测试虚拟CAN环境下的电机控制
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import math

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        
        # 创建努力控制发布器
        self.effort_publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )
        
        # 创建关节状态订阅器
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 当前关节状态
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0]
        self.joint_efforts = [0.0, 0.0, 0.0, 0.0]
        
        # 创建定时器进行周期性测试
        self.test_timer = self.create_timer(0.1, self.test_callback)
        
        # 测试参数
        self.test_start_time = time.time()
        self.test_phase = 0
        
        self.get_logger().info("电机测试器已启动")
    
    def joint_state_callback(self, msg):
        """关节状态回调"""
        if len(msg.position) >= 4:
            self.joint_positions = list(msg.position[:4])
        if len(msg.velocity) >= 4:
            self.joint_velocities = list(msg.velocity[:4])
        if len(msg.effort) >= 4:
            self.joint_efforts = list(msg.effort[:4])
    
    def test_callback(self):
        """测试回调函数"""
        current_time = time.time() - self.test_start_time
        
        # 创建命令消息
        cmd_msg = Float64MultiArray()
        
        if current_time < 5.0:
            # 阶段1：保持静止 (0-5秒)
            cmd_msg.data = [0.0, 0.0, 0.0, 0.0]
            if self.test_phase != 1:
                self.get_logger().info("测试阶段1: 保持静止")
                self.test_phase = 1
                
        elif current_time < 10.0:
            # 阶段2：单个电机测试 (5-10秒)
            cmd_msg.data = [1000.0, 0.0, 0.0, 0.0]  # 只测试motor1
            if self.test_phase != 2:
                self.get_logger().info("测试阶段2: 单个电机测试 (motor1)")
                self.test_phase = 2
                
        elif current_time < 15.0:
            # 阶段3：正弦波测试 (10-15秒)
            amplitude = 2000.0
            frequency = 0.5  # Hz
            phase = 2 * math.pi * frequency * (current_time - 10.0)
            
            cmd_msg.data = [
                amplitude * math.sin(phase),
                amplitude * math.sin(phase + math.pi/2),
                amplitude * math.sin(phase + math.pi),
                amplitude * math.sin(phase + 3*math.pi/2)
            ]
            if self.test_phase != 3:
                self.get_logger().info("测试阶段3: 正弦波测试")
                self.test_phase = 3
                
        elif current_time < 20.0:
            # 阶段4：方波测试 (15-20秒)
            period = 2.0  # 2秒周期
            phase_time = (current_time - 15.0) % period
            
            if phase_time < period / 2:
                cmd_msg.data = [3000.0, -3000.0, 3000.0, -3000.0]
            else:
                cmd_msg.data = [-3000.0, 3000.0, -3000.0, 3000.0]
                
            if self.test_phase != 4:
                self.get_logger().info("测试阶段4: 方波测试")
                self.test_phase = 4
                
        else:
            # 阶段5：停止 (20秒后)
            cmd_msg.data = [0.0, 0.0, 0.0, 0.0]
            if self.test_phase != 5:
                self.get_logger().info("测试阶段5: 停止")
                self.test_phase = 5
        
        # 发布命令
        self.effort_publisher.publish(cmd_msg)
        
        # 定期打印状态
        if int(current_time * 10) % 10 == 0:  # 每秒打印一次
            self.get_logger().info(
                f"时间: {current_time:.1f}s | "
                f"位置: [{', '.join([f'{p:.2f}' for p in self.joint_positions])}] | "
                f"速度: [{', '.join([f'{v:.2f}' for v in self.joint_velocities])}] | "
                f"努力: [{', '.join([f'{e:.0f}' for e in self.joint_efforts])}]"
            )

def main(args=None):
    rclpy.init(args=args)
    
    motor_tester = MotorTester()
    
    try:
        rclpy.spin(motor_tester)
    except KeyboardInterrupt:
        motor_tester.get_logger().info("收到中断信号，退出测试")
    finally:
        motor_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 