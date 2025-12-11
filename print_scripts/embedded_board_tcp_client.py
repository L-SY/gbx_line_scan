#!/usr/bin/env python3
"""
嵌入式主板网口通信客户端
根据文档说明实现TCP网口通信功能

支持的数据格式：
1. 简易格式: xxx [Rn]xxx\r\n
2. JSON格式: {regNum=n,content=xxx}\r\n
3. 特殊符号兼容格式: 01内容1 | 11内容2 | 12内容3\r\n

支持的远程指令：
- 寄存器20: 打印控制 (start/pause/print/continue/stop/resetCount)
- 寄存器22: 信息获取 (inkVolume/error)
- 寄存器24: 文件切换
"""

import socket
import time
import threading
from typing import Optional, Callable
from enum import Enum


class DataFormat(Enum):
    """数据格式枚举"""
    SIMPLE = "simple"          # 简易格式
    JSON = "json"              # JSON格式
    JSON_MULTI = "json_multi"  # 多寄存器JSON格式
    SPECIAL = "special"        # 特殊符号兼容格式


class PrintCommand(Enum):
    """打印控制指令"""
    START = "start"
    PAUSE = "pause"
    PRINT = "print"
    CONTINUE = "continue"
    STOP = "stop"
    RESET_COUNT = "resetCount"


class InfoCommand(Enum):
    """信息获取指令"""
    INK_VOLUME = "inkVolume"
    ERROR = "error"


class EmbeddedBoardTCPClient:
    """嵌入式主板TCP通信客户端"""
    
    def __init__(self, host: str = "192.168.1.19", port: int = 8989, timeout: float = 5.0):
        """
        初始化TCP客户端
        
        Args:
            host: 设备IP地址
            port: 端口号
            timeout: 连接超时时间（秒）
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None
        self.connected = False
        self.receive_thread: Optional[threading.Thread] = None
        self.running = False
        self.receive_callback: Optional[Callable[[str], None]] = None
        
    def connect(self) -> bool:
        """
        连接到设备
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"成功连接到设备 {self.host}:{self.port}")
            
            # 启动接收线程
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            
            return True
        except socket.timeout:
            print(f"连接超时: {self.host}:{self.port}")
            return False
        except socket.error as e:
            print(f"连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        self.running = False
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        print("已断开连接")
    
    def _receive_loop(self):
        """接收数据循环（在独立线程中运行）"""
        buffer = b""
        while self.running and self.connected:
            try:
                if self.socket:
                    self.socket.settimeout(1.0)  # 设置接收超时，以便检查running标志
                    data = self.socket.recv(4096)
                    if not data:
                        break
                    buffer += data
                    
                    # 处理完整的数据包（以\r\n结尾）
                    while b"\r\n" in buffer:
                        line, buffer = buffer.split(b"\r\n", 1)
                        message = line.decode('utf-8', errors='ignore')
                        if self.receive_callback:
                            self.receive_callback(message)
                        else:
                            print(f"收到数据: {message}")
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"接收数据错误: {e}")
                break
    
    def set_receive_callback(self, callback: Callable[[str], None]):
        """
        设置接收数据回调函数
        
        Args:
            callback: 回调函数，参数为接收到的消息字符串
        """
        self.receive_callback = callback
    
    def send(self, data: str) -> bool:
        """
        发送数据（自动添加\r\n结尾）
        
        Args:
            data: 要发送的数据字符串
            
        Returns:
            True if send successful, False otherwise
        """
        if not self.connected or not self.socket:
            print("未连接到设备")
            return False
        
        try:
            # 确保数据以\r\n结尾
            if not data.endswith("\r\n"):
                data += "\r\n"
            
            self.socket.sendall(data.encode('utf-8'))
            print(f"发送数据: {data.strip()}")
            return True
        except Exception as e:
            print(f"发送数据失败: {e}")
            return False
    
    # ========== 数据格式发送方法 ==========
    
    def _validate_register_num(self, register_num: int, allow_special: bool = False) -> bool:
        """
        验证寄存器号是否有效
        
        Args:
            register_num: 寄存器号
            allow_special: 是否允许特殊寄存器（20, 22, 24）
            
        Returns:
            True if valid
        """
        if register_num < 1:
            print(f"寄存器号必须大于0，当前值: {register_num}")
            return False
        
        # 普通数据寄存器：1-16
        if 1 <= register_num <= 16:
            return True
        
        # 特殊功能寄存器：20, 22, 24
        if allow_special and register_num in [20, 22, 24]:
            return True
        
        if allow_special:
            print(f"寄存器号无效。支持范围: 1-16（数据寄存器）或 20,22,24（特殊寄存器），当前值: {register_num}")
        else:
            print(f"寄存器号必须在1-16之间，当前值: {register_num}")
        
        return False
    
    def send_simple_format(self, register_num: int, content: str) -> bool:
        """
        发送简易格式数据: xxx [Rn]xxx\r\n
        
        Args:
            register_num: 寄存器号 (1-16，普通数据寄存器)
            content: 传输数据
            
        Returns:
            True if send successful
        """
        if not self._validate_register_num(register_num, allow_special=False):
            return False
        
        data = f"{content} [R{register_num}]{content}\r\n"
        return self.send(data)
    
    def send_json_format(self, register_num: int, content: str) -> bool:
        """
        发送JSON格式数据: {regNum=n,content=xxx}\r\n
        
        Args:
            register_num: 寄存器号 (1-16 数据寄存器，或 20,22,24 特殊寄存器)
            content: 传输数据（非数字需要加引号）
            
        Returns:
            True if send successful
        """
        if not self._validate_register_num(register_num, allow_special=True):
            return False
        
        # 判断content是否为数字，如果不是则加引号
        try:
            float(content)
            # 是数字，不需要引号
            data = f"{{regNum={register_num},content={content}}}\r\n"
        except ValueError:
            # 不是数字，需要加引号
            data = f"{{regNum={register_num},content=\"{content}\"}}\r\n"
        
        return self.send(data)
    
    def send_json_multi_format(self, registers: list[tuple[int, str]]) -> bool:
        """
        发送多寄存器JSON格式: {regNum=n,content=xxx}{regNum=n,content=xxx}...\r\n
        
        Args:
            registers: 寄存器列表，每个元素为(寄存器号, 内容)元组
                      寄存器号支持: 1-16（数据寄存器）或 20,22,24（特殊寄存器）
            
        Returns:
            True if send successful
        """
        data_parts = []
        for reg_num, content in registers:
            if not self._validate_register_num(reg_num, allow_special=True):
                continue
            
            try:
                float(content)
                data_parts.append(f"{{regNum={reg_num},content={content}}}")
            except ValueError:
                data_parts.append(f"{{regNum={reg_num},content=\"{content}\"}}")
        
        if not data_parts:
            return False
        
        data = "".join(data_parts) + "\r\n"
        return self.send(data)
    
    def send_special_format(self, registers: list[tuple[int, str]]) -> bool:
        """
        发送特殊符号兼容格式: 01内容1 | 11内容2 | 12内容3\r\n
        
        Args:
            registers: 寄存器列表，每个元素为(寄存器号, 内容)元组
                      寄存器号支持: 1-16（数据寄存器）或 20,22,24（特殊寄存器）
            
        Returns:
            True if send successful
        """
        data_parts = []
        for reg_num, content in registers:
            if not self._validate_register_num(reg_num, allow_special=True):
                continue
            
            # 寄存器号格式化为两位数字（01-99）
            reg_str = f"{reg_num:02d}"
            data_parts.append(f"{reg_str}{content}")
        
        if not data_parts:
            return False
        
        data = " | ".join(data_parts) + "\r\n"
        return self.send(data)
    
    # ========== 远程指令控制方法 ==========
    
    def control_print(self, command: PrintCommand) -> bool:
        """
        控制打印（寄存器20）
        
        Args:
            command: 打印控制指令
            
        Returns:
            True if send successful
        """
        return self.send_json_format(20, command.value)
    
    def get_info(self, command: InfoCommand) -> bool:
        """
        获取信息（寄存器22）
        
        Args:
            command: 信息获取指令
            
        Returns:
            True if send successful
        """
        return self.send_json_format(22, command.value)
    
    def switch_file(self, filename: str) -> bool:
        """
        切换文件（寄存器24）
        
        Args:
            filename: 文件名（不包含.msgx后缀）
            
        Returns:
            True if send successful
        """
        return self.send_json_format(24, filename)
    
    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.disconnect()


# ========== 使用示例 ==========

def example_basic_usage():
    """基本使用示例"""
    print("\n=== 基本使用示例 ===")
    
    # 创建客户端并连接
    client = EmbeddedBoardTCPClient(host="192.168.1.19", port=8989)
    
    if not client.connect():
        print("连接失败，请检查网络配置")
        return
    
    try:
        # 示例1: 发送简易格式数据到寄存器1
        client.send_simple_format(1, "Hello")
        time.sleep(0.5)
        
        # 示例2: 发送JSON格式数据到寄存器2
        client.send_json_format(2, "123.45")
        time.sleep(0.5)
        
        # 示例3: 发送JSON格式文本数据到寄存器3
        client.send_json_format(3, "Test Message")
        time.sleep(0.5)
        
        # 示例4: 发送多寄存器JSON格式
        client.send_json_multi_format([
            (1, "Data1"),
            (2, "Data2"),
            (3, "123")
        ])
        time.sleep(0.5)
        
        # 示例5: 发送特殊符号兼容格式
        client.send_special_format([
            (1, "内容1!@#$%^"),
            (2, "内容2"),
            (3, "内容3")
        ])
        time.sleep(0.5)
        
    finally:
        client.disconnect()


def example_remote_commands():
    """远程指令控制示例"""
    print("\n=== 远程指令控制示例 ===")
    
    client = EmbeddedBoardTCPClient(host="192.168.1.19", port=8989)
    
    if not client.connect():
        print("连接失败，请检查网络配置")
        return
    
    try:
        # 设置接收回调
        def on_receive(message: str):
            print(f"收到响应: {message}")
            # 解析响应（例如: inkVolume=85 或 error=xxx）
            if "=" in message:
                key, value = message.split("=", 1)
                print(f"  -> {key}: {value}")
        
        client.set_receive_callback(on_receive)
        
        # 控制打印
        print("\n--- 打印控制 ---")
        client.control_print(PrintCommand.START)
        time.sleep(1)
        
        client.control_print(PrintCommand.PAUSE)
        time.sleep(1)
        
        client.control_print(PrintCommand.CONTINUE)
        time.sleep(1)
        
        client.control_print(PrintCommand.STOP)
        time.sleep(1)
        
        # 获取信息
        print("\n--- 信息获取 ---")
        client.get_info(InfoCommand.INK_VOLUME)
        time.sleep(1)
        
        client.get_info(InfoCommand.ERROR)
        time.sleep(1)
        
        # 文件切换
        print("\n--- 文件切换 ---")
        client.switch_file("new_file")
        time.sleep(1)
        
    finally:
        client.disconnect()


def example_with_context_manager():
    """使用上下文管理器示例"""
    print("\n=== 上下文管理器示例 ===")
    
    with EmbeddedBoardTCPClient(host="192.168.1.19", port=8989) as client:
        if client.connected:
            # 发送数据
            client.send_json_format(1, "Context Manager Test")
            time.sleep(1)
            
            # 重置计数
            client.control_print(PrintCommand.RESET_COUNT)
            time.sleep(1)
    # 自动断开连接


if __name__ == "__main__":
    print("嵌入式主板TCP通信客户端示例")
    print("=" * 50)
    
    # 注意：实际使用时需要根据设备IP和端口进行配置
    # 可以通过命令行参数或配置文件设置
    
    import sys
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "basic":
            example_basic_usage()
        elif sys.argv[1] == "commands":
            example_remote_commands()
        elif sys.argv[1] == "context":
            example_with_context_manager()
        else:
            print("用法: python embedded_board_tcp_client.py [basic|commands|context]")
    else:
        # 默认运行所有示例
        example_basic_usage()
        time.sleep(2)
        example_remote_commands()
        time.sleep(2)
        example_with_context_manager()


