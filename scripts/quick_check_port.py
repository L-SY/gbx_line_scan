#!/usr/bin/env python3
"""
快速检查端口是否开放
"""

import socket
import sys


def check_port(host: str, port: int, timeout: float = 3.0):
    """检查端口是否开放"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except:
        return False


if __name__ == "__main__":
    if len(sys.argv) >= 3:
        host = sys.argv[1]
        port = int(sys.argv[2])
    else:
        host = "192.168.1.100"
        port = 8989
    
    print(f"检查 {host}:{port}...")
    
    if check_port(host, port):
        print(f"✅ 端口 {port} 开放")
    else:
        print(f"❌ 端口 {port} 未开放")
        print("\n可能原因：")
        print("  1. 设备未启用网络通信功能")
        print("  2. 端口号配置错误")
        print("  3. LUA脚本未启用")
        print("  4. 设备TCP服务未启动")
        print("\n运行完整诊断: python diagnose_connection.py")


