#!/usr/bin/env python3
"""
连接诊断工具
用于诊断TCP连接问题
"""

import socket
import sys
import subprocess
import time
from typing import List, Tuple


def check_port(host: str, port: int, timeout: float = 3.0) -> Tuple[bool, str]:
    """
    检查TCP端口是否开放
    
    Returns:
        (是否开放, 错误信息)
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((host, port))
        sock.close()
        
        if result == 0:
            return True, "端口开放"
        elif result == 111:
            return False, "连接被拒绝 (Connection refused) - 端口未开放或服务未运行"
        elif result == 110:
            return False, "连接超时 (Connection timeout)"
        elif result == 113:
            return False, "无路由到主机 (No route to host)"
        else:
            return False, f"连接失败 (错误代码: {result})"
    except socket.gaierror:
        return False, "无法解析主机名"
    except Exception as e:
        return False, f"检查失败: {e}"


def check_common_ports(host: str) -> List[Tuple[int, bool, str]]:
    """检查常见端口"""
    common_ports = [8989, 8080, 80, 443, 23, 22, 8888, 9999]
    results = []
    
    print(f"\n检查常见端口...")
    for port in common_ports:
        is_open, msg = check_port(host, port, timeout=2.0)
        status = "✓ 开放" if is_open else "✗ 关闭"
        results.append((port, is_open, msg))
        print(f"  端口 {port:5d}: {status} - {msg}")
    
    return results


def test_ping(host: str) -> bool:
    """测试ping"""
    try:
        result = subprocess.run(
            ['ping', '-c', '3', host],
            capture_output=True,
            text=True,
            timeout=10
        )
        return result.returncode == 0
    except:
        return False


def scan_port_range(host: str, start_port: int = 8980, end_port: int = 9000) -> List[int]:
    """扫描端口范围，找出开放的端口"""
    print(f"\n扫描端口范围 {start_port}-{end_port}...")
    open_ports = []
    
    for port in range(start_port, end_port + 1):
        is_open, _ = check_port(host, port, timeout=1.0)
        if is_open:
            open_ports.append(port)
            print(f"  ✓ 发现开放端口: {port}")
        if port % 5 == 0:
            print(f"  扫描进度: {port}/{end_port}", end='\r')
    
    print()  # 换行
    return open_ports


def test_telnet_style(host: str, port: int):
    """尝试telnet风格的连接测试"""
    print(f"\n尝试详细连接测试...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        
        print(f"  正在连接到 {host}:{port}...")
        start_time = time.time()
        result = sock.connect_ex((host, port))
        elapsed = time.time() - start_time
        
        if result == 0:
            print(f"  ✓ 连接成功！(耗时: {elapsed:.3f}秒)")
            print(f"  ✓ 端口 {port} 是开放的")
            
            # 尝试发送测试数据
            try:
                test_data = b"test\r\n"
                sock.sendall(test_data)
                print(f"  ✓ 成功发送测试数据")
                
                # 尝试接收响应
                sock.settimeout(2.0)
                try:
                    response = sock.recv(1024)
                    if response:
                        print(f"  ✓ 收到响应: {response.decode('utf-8', errors='ignore')}")
                    else:
                        print(f"  ⚠ 未收到响应（可能正常，取决于设备配置）")
                except socket.timeout:
                    print(f"  ⚠ 接收超时（可能正常，设备可能不主动发送数据）")
            except Exception as e:
                print(f"  ⚠ 发送数据时出错: {e}")
            
            sock.close()
            return True
        else:
            print(f"  ✗ 连接失败 (错误代码: {result})")
            print(f"  ✗ 耗时: {elapsed:.3f}秒")
            sock.close()
            return False
            
    except Exception as e:
        print(f"  ✗ 连接测试失败: {e}")
        return False


def diagnose(host: str, port: int = 8989):
    """完整诊断流程"""
    print("=" * 60)
    print("TCP连接诊断工具")
    print("=" * 60)
    print(f"目标设备: {host}:{port}")
    print()
    
    # 1. 测试ping
    print("步骤1: 测试网络连通性 (ping)")
    if test_ping(host):
        print("  ✓ Ping成功 - 网络层连通正常")
    else:
        print("  ✗ Ping失败 - 请检查网络连接")
        return
    print()
    
    # 2. 检查目标端口
    print(f"步骤2: 检查目标端口 {port}")
    is_open, msg = check_port(host, port, timeout=5.0)
    if is_open:
        print(f"  ✓ 端口 {port} 开放")
        print(f"  ✓ {msg}")
        print("\n✅ 端口正常，可以尝试连接！")
        test_telnet_style(host, port)
    else:
        print(f"  ✗ 端口 {port} 未开放")
        print(f"  ✗ {msg}")
        print()
    
    # 3. 检查常见端口
    if not is_open:
        results = check_common_ports(host)
        open_ports = [p for p, open, _ in results if open]
        
        if open_ports:
            print(f"\n⚠️  发现其他开放端口: {open_ports}")
            print(f"   可能设备使用了不同的端口号")
        else:
            print(f"\n⚠️  未发现任何开放端口")
        print()
    
    # 4. 端口扫描（如果目标端口未开放）
    if not is_open:
        print("步骤3: 扫描端口范围（查找可能的端口）")
        response = input("  是否扫描端口范围 8980-9000? (y/n, 默认n): ").strip().lower()
        if response == 'y':
            open_ports = scan_port_range(host, 8980, 9000)
            if open_ports:
                print(f"\n✅ 发现开放端口: {open_ports}")
                print(f"   建议检查设备配置，确认正确的端口号")
            else:
                print(f"\n⚠️  在8980-9000范围内未发现开放端口")
        print()
    
    # 5. 诊断建议
    print("=" * 60)
    print("诊断建议")
    print("=" * 60)
    
    if not is_open:
        print("\n🔍 可能的原因和解决方案：")
        print()
        print("1. 【设备未启用网络通信】")
        print("   → 检查设备硬件设置：")
        print("     - 通信方式是否选择了'网络'（而不是'串口'）")
        print("     - 网络参数是否正确配置")
        print()
        print("2. 【端口号配置错误】")
        print("   → 检查设备硬件设置中的端口号")
        print("   → 确认端口号是否为 8989（或其他值）")
        print("   → 如果设备显示其他端口号，请在代码中修改")
        print()
        print("3. 【LUA脚本未启用】")
        print("   → 在设备硬件设置中：")
        print("     - 找到'当前脚本'选项")
        print("     - 选择并启用相应的脚本（如 getString_json.lua）")
        print("     - 确保脚本状态为'启用'")
        print()
        print("4. 【设备TCP服务未启动】")
        print("   → 尝试重启设备")
        print("   → 检查设备是否有网络服务状态指示")
        print()
        print("5. 【防火墙阻止】")
        print("   → 检查设备端是否有防火墙设置")
        print("   → 检查PC端防火墙是否阻止了连接")
        print()
        print("6. 【设备需要特定触发】")
        print("   → 某些设备可能需要先发送特定数据才能建立连接")
        print("   → 查看设备文档是否有特殊要求")
        print()
    
    print("=" * 60)


if __name__ == "__main__":
    if len(sys.argv) >= 3:
        host = sys.argv[1]
        port = int(sys.argv[2])
    elif len(sys.argv) == 2:
        host = sys.argv[1]
        port = 8989
    else:
        host = "192.168.1.120"
        port = 8989
        print(f"使用默认配置: {host}:{port}")
        print("如需指定IP和端口，请使用: python diagnose_connection.py <IP> [PORT]")
        print()
    
    diagnose(host, port)


