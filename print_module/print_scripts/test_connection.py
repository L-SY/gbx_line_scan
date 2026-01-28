#!/usr/bin/env python3
"""
快速测试连接脚本
用于快速验证设备连接是否正常
"""

import sys
import time
from embedded_board_tcp_client import EmbeddedBoardTCPClient, InfoCommand


def test_connection(host: str, port: int):
    """测试连接"""
    print(f"正在测试连接到 {host}:{port}...")
    print("-" * 50)
    
    client = EmbeddedBoardTCPClient(host=host, port=port, timeout=3.0)
    
    # 设置接收回调
    received = []
    def on_receive(message: str):
        received.append(message)
        print(f"  ✓ 收到响应: {message}")
    
    client.set_receive_callback(on_receive)
    
    # 尝试连接
    if not client.connect():
        print("❌ 连接失败")
        print("\n可能的原因：")
        print("  1. 设备未开机或未连接到网络")
        print("  2. IP地址或端口号错误")
        print("  3. 防火墙阻止连接")
        print("  4. 设备未启用网络通信")
        return False
    
    print("✅ 连接成功")
    print()
    
    try:
        # 测试1: 连通性测试（发送到寄存器1）
        print("测试1: 连通性测试")
        client.send("test_connectivity\r\n")
        time.sleep(1)
        
        # 测试2: JSON格式数据
        print("\n测试2: 发送JSON格式数据到寄存器1")
        client.send_json_format(1, "test_data")
        time.sleep(1)
        
        # 测试3: 获取墨量信息
        print("\n测试3: 查询墨量信息")
        client.get_info(InfoCommand.INK_VOLUME)
        time.sleep(2)
        
        print("\n" + "-" * 50)
        if received:
            print(f"✅ 测试完成，共收到 {len(received)} 条响应")
        else:
            print("⚠️  测试完成，但未收到响应（可能设备未配置回传）")
        
        return True
        
    except Exception as e:
        print(f"❌ 测试过程中出错: {e}")
        return False
    finally:
        client.disconnect()


if __name__ == "__main__":
    # 从命令行参数获取IP和端口，或使用默认值
    if len(sys.argv) >= 3:
        host = sys.argv[1]
        port = int(sys.argv[2])
    else:
        host = "192.168.1.120"
        port = 8989
        print(f"使用默认配置: {host}:{port}")
        print("如需指定IP和端口，请使用: python test_connection.py <IP> <PORT>")
        print()
    
    test_connection(host, port)

