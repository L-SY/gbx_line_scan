#!/usr/bin/env python3
"""
虚拟 CAN 信号模拟器
模拟 RM 电机的 CAN 反馈信号用于测试
"""

import struct
import time
import socket
import threading
import math
import argparse

class VirtualRMMotor:
    """虚拟 RM 电机模拟器"""
    
    def __init__(self, motor_id, rx_id, tx_id, name="motor"):
        self.motor_id = motor_id
        self.rx_id = rx_id  # 电机反馈ID
        self.tx_id = tx_id  # 控制命令ID
        self.name = name
        
        # 电机状态
        self.encoder = 0        # 编码器值 (0-8191)
        self.velocity = 0       # 速度 (RPM)
        self.current = 0        # 电流
        self.temperature = 25   # 温度
        
        # 物理模拟参数
        self.position = 0.0     # 当前位置 (弧度)
        self.target_current = 0 # 目标电流
        self.max_current = 10000
        
        # 模拟参数
        self.inertia = 0.1      # 惯性
        self.damping = 0.05     # 阻尼
        self.dt = 0.001         # 时间步长
        
        print(f"创建虚拟电机: {name} (ID: {motor_id}, RX: 0x{rx_id:X}, TX: 0x{tx_id:X})")
    
    def update_physics(self):
        """更新物理模拟"""
        # 简单的物理模拟：电流产生力矩，推动电机转动
        torque = self.current * 0.001  # 简化的力矩计算
        acceleration = (torque - self.damping * self.velocity) / self.inertia
        
        self.velocity += acceleration * self.dt
        self.position += self.velocity * self.dt
        
        # 更新编码器值 (0-8191)
        self.encoder = int((self.position * 8192 / (2 * math.pi)) % 8192)
        
        # 模拟电流响应目标电流
        current_error = self.target_current - self.current
        self.current += current_error * 0.1  # 简单的一阶响应
        
        # 温度模拟（基于电流）
        self.temperature = 25 + abs(self.current) / 500
    
    def process_command(self, data):
        """处理控制命令"""
        if len(data) >= 8:
            # RM 电机命令格式解析
            motor_index = self.motor_id % 4
            current_cmd = struct.unpack('>h', data[motor_index*2:(motor_index+1)*2])[0]
            self.target_current = current_cmd
            print(f"{self.name}: 收到控制命令，目标电流: {current_cmd}")
    
    def get_feedback_frame(self):
        """生成反馈CAN帧"""
        # RM 电机反馈格式：
        # Byte 0-1: 编码器角度 (0-8191)
        # Byte 2-3: 转速 (RPM) 
        # Byte 4-5: 力矩电流
        # Byte 6: 温度
        # Byte 7: 保留
        
        encoder_int = int(self.encoder) & 0x1FFF
        velocity_int = int(self.velocity) & 0xFFFF
        current_int = int(self.current) & 0xFFFF
        temp_int = int(self.temperature) & 0xFF
        
        data = struct.pack('>HHHBB', 
                          encoder_int,    # 编码器
                          velocity_int,   # 速度
                          current_int,    # 电流
                          temp_int,       # 温度
                          0)              # 保留位
        
        return data

class CanSimulator:
    """CAN 总线模拟器"""
    
    def __init__(self, interface='vcan0'):
        self.interface = interface
        self.can_socket = None
        self.running = False
        
        # 创建虚拟电机 (模拟4个RM2006电机)
        self.motors = {
            0x201: VirtualRMMotor(0, 0x201, 0x200, "motor1"),
            0x202: VirtualRMMotor(1, 0x202, 0x200, "motor2"),
            0x203: VirtualRMMotor(2, 0x203, 0x200, "motor3"),
            0x204: VirtualRMMotor(3, 0x204, 0x200, "motor4"),
        }
        
        self.setup_socket()
    
    def setup_socket(self):
        """设置CAN socket"""
        try:
            # 创建 CAN socket
            self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_socket.bind((self.interface,))
            print(f"CAN 模拟器已绑定到接口: {self.interface}")
        except Exception as e:
            print(f"创建CAN socket失败: {e}")
            raise
    
    def send_frame(self, can_id, data):
        """发送CAN帧"""
        if self.can_socket is None:
            return
        try:
            # CAN帧格式: ID (4字节) + DLC (1字节) + 数据 (最多8字节)
            frame = struct.pack("=IB3x8s", can_id, len(data), data.ljust(8, b'\x00'))
            self.can_socket.send(frame)
        except Exception as e:
            print(f"发送CAN帧失败: {e}")
    
    def receive_frame(self):
        """接收CAN帧"""
        if self.can_socket is None:
            return None, None
        try:
            frame = self.can_socket.recv(16)
            can_id, dlc = struct.unpack("=IB3x", frame[:8])
            data = frame[8:8+dlc]
            return can_id, data
        except socket.timeout:
            return None, None
        except Exception as e:
            print(f"接收CAN帧失败: {e}")
            return None, None
    
    def handle_command(self, can_id, data):
        """处理控制命令"""
        if can_id == 0x200:  # 电机1-4控制命令
            for motor in self.motors.values():
                if motor.tx_id == can_id:
                    motor.process_command(data)
        elif can_id == 0x1FF:  # 电机5-8控制命令
            print("接收到电机5-8控制命令（暂未实现）")
    
    def run(self):
        """运行模拟器"""
        if self.can_socket is None:
            print("CAN socket未初始化")
            return
            
        self.running = True
        print("CAN 模拟器开始运行...")
        
        # 设置接收超时
        self.can_socket.settimeout(0.01)
        
        last_feedback_time = time.time()
        
        while self.running:
            try:
                # 接收控制命令
                can_id, data = self.receive_frame()
                if can_id is not None:
                    self.handle_command(can_id, data)
                
                # 定期发送反馈 (1000Hz)
                current_time = time.time()
                if current_time - last_feedback_time >= 0.001:
                    for motor in self.motors.values():
                        motor.update_physics()
                        feedback_data = motor.get_feedback_frame()
                        self.send_frame(motor.rx_id, feedback_data)
                    
                    last_feedback_time = current_time
                
                time.sleep(0.0001)  # 避免占用过多CPU
                
            except KeyboardInterrupt:
                print("\n收到中断信号，停止模拟器...")
                break
            except Exception as e:
                print(f"模拟器运行错误: {e}")
                time.sleep(0.1)
        
        self.stop()
    
    def stop(self):
        """停止模拟器"""
        self.running = False
        if self.can_socket:
            self.can_socket.close()
        print("CAN 模拟器已停止")

def main():
    parser = argparse.ArgumentParser(description='虚拟CAN信号模拟器')
    parser.add_argument('--interface', '-i', default='vcan0', 
                      help='CAN接口名称 (默认: vcan0)')
    parser.add_argument('--motors', '-m', type=int, default=4,
                      help='模拟电机数量 (默认: 4)')
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("虚拟CAN信号模拟器")
    print("=" * 50)
    print(f"CAN接口: {args.interface}")
    print(f"模拟电机数量: {args.motors}")
    print("按 Ctrl+C 停止模拟器")
    print("=" * 50)
    
    try:
        simulator = CanSimulator(args.interface)
        simulator.run()
    except Exception as e:
        print(f"模拟器启动失败: {e}")

if __name__ == "__main__":
    main() 