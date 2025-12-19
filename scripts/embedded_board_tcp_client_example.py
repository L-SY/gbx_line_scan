#!/usr/bin/env python3
"""
嵌入式主板TCP通信客户端 - 实际使用示例
演示如何在实际场景中使用TCP客户端与设备通信
"""

import sys
import time
from embedded_board_tcp_client import (
    EmbeddedBoardTCPClient,
    PrintCommand,
    InfoCommand
)


def main():
    """主函数"""
    # 配置设备IP和端口（根据实际情况修改）
    DEVICE_IP = "192.168.1.120"  # 设备IP地址
    DEVICE_PORT = 8989           # 设备端口号
    
    print("=" * 60)
    print("嵌入式主板TCP通信客户端")
    print("=" * 60)
    print(f"设备地址: {DEVICE_IP}:{DEVICE_PORT}")
    print()
    
    # 创建客户端
    client = EmbeddedBoardTCPClient(host=DEVICE_IP, port=DEVICE_PORT, timeout=5.0)
    
    # 设置接收数据回调
    def on_receive(message: str):
        """接收数据回调函数"""
        print(f"[接收] {message}")
    
    client.set_receive_callback(on_receive)
    
    # 连接到设备
    print("正在连接设备...")
    if not client.connect():
        print("❌ 连接失败！")
        print("\n可能的原因：")
        print("1. 设备未开机或未连接到网络")
        print("2. IP地址或端口号配置错误")
        print("3. 防火墙阻止了连接")
        print("4. 设备未启用网络通信功能")
        print("\n请检查：")
        print(f"  - 设备IP地址是否为: {DEVICE_IP}")
        print(f"  - 设备端口号是否为: {DEVICE_PORT}")
        print("  - 设备硬件设置中是否选择了'网络'通信方式")
        print("  - 设备是否已启用相应的脚本（如checkConnectivity.lua）")
        return
    
    print("✅ 连接成功！")
    print()
    
    try:
        # 交互式菜单
        while True:
            print("\n" + "=" * 60)
            print("请选择操作：")
            print("  1. 发送数据到寄存器（简易格式）")
            print("  2. 发送数据到寄存器（JSON格式）")
            print("  3. 发送数据到多个寄存器（JSON格式）")
            print("  4. 发送数据（特殊符号兼容格式）")
            print("  5. 控制打印")
            print("  6. 获取设备信息")
            print("  7. 切换文件")
            print("  8. 测试连通性")
            print("  0. 退出")
            print("=" * 60)
            
            choice = input("请输入选项 (0-8): ").strip()
            
            if choice == "0":
                print("退出程序")
                break
            
            elif choice == "1":
                # 简易格式
                try:
                    reg = int(input("寄存器号 (1-16): "))
                    content = input("内容: ")
                    if client.send_simple_format(reg, content):
                        print("✅ 发送成功")
                except ValueError:
                    print("❌ 输入错误")
            
            elif choice == "2":
                # JSON格式
                try:
                    reg = int(input("寄存器号 (1-16): "))
                    content = input("内容: ")
                    if client.send_json_format(reg, content):
                        print("✅ 发送成功")
                except ValueError:
                    print("❌ 输入错误")
            
            elif choice == "3":
                # 多寄存器JSON格式
                try:
                    count = int(input("寄存器数量: "))
                    registers = []
                    for i in range(count):
                        reg = int(input(f"  寄存器{i+1}号 (1-16): "))
                        content = input(f"  寄存器{i+1}内容: ")
                        registers.append((reg, content))
                    if client.send_json_multi_format(registers):
                        print("✅ 发送成功")
                except ValueError:
                    print("❌ 输入错误")
            
            elif choice == "4":
                # 特殊符号兼容格式
                try:
                    count = int(input("寄存器数量: "))
                    registers = []
                    for i in range(count):
                        reg = int(input(f"  寄存器{i+1}号 (1-16): "))
                        content = input(f"  寄存器{i+1}内容: ")
                        registers.append((reg, content))
                    if client.send_special_format(registers):
                        print("✅ 发送成功")
                except ValueError:
                    print("❌ 输入错误")
            
            elif choice == "5":
                # 控制打印
                print("\n打印控制选项：")
                print("  1. start - 开启喷印")
                print("  2. pause - 暂停喷印")
                print("  3. print - 强制喷印")
                print("  4. continue - 继续喷印")
                print("  5. stop - 退出喷印")
                print("  6. resetCount - 重置计数")
                cmd_choice = input("请选择 (1-6): ").strip()
                
                commands = {
                    "1": PrintCommand.START,
                    "2": PrintCommand.PAUSE,
                    "3": PrintCommand.PRINT,
                    "4": PrintCommand.CONTINUE,
                    "5": PrintCommand.STOP,
                    "6": PrintCommand.RESET_COUNT
                }
                
                if cmd_choice in commands:
                    if client.control_print(commands[cmd_choice]):
                        print("✅ 发送成功")
                else:
                    print("❌ 无效选项")
            
            elif choice == "6":
                # 获取设备信息
                print("\n信息获取选项：")
                print("  1. inkVolume - 查看墨量")
                print("  2. error - 获取报警信息")
                info_choice = input("请选择 (1-2): ").strip()
                
                if info_choice == "1":
                    if client.get_info(InfoCommand.INK_VOLUME):
                        print("✅ 发送成功，等待响应...")
                elif info_choice == "2":
                    if client.get_info(InfoCommand.ERROR):
                        print("✅ 发送成功，等待响应...")
                else:
                    print("❌ 无效选项")
            
            elif choice == "7":
                # 切换文件
                filename = input("文件名（不含.msgx后缀）: ").strip()
                if filename:
                    if client.switch_file(filename):
                        print("✅ 发送成功")
                else:
                    print("❌ 文件名不能为空")
            
            elif choice == "8":
                # 测试连通性
                print("发送连通性测试数据...")
                # 使用checkConnectivity.lua脚本，数据会传输到1号寄存器
                if client.send("test_connectivity\r\n"):
                    print("✅ 发送成功，等待响应...")
                    time.sleep(1)
            
            else:
                print("❌ 无效选项，请重新选择")
            
            time.sleep(0.5)  # 短暂延迟
    
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
    finally:
        client.disconnect()
        print("已断开连接")


if __name__ == "__main__":
    main()


