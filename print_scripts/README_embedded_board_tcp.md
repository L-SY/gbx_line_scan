# 嵌入式主板网口通信实现说明

## 概述

本文档说明如何实现电脑与嵌入式主板的网口通信。根据设备文档，设备支持通过TCP协议进行网络通信。

## 准备工作

### 1. 硬件准备
- 网线
- 路由器（根据需求选择）或直连
- 确保设备已连接到网络或与PC直连

### 2. 设备配置

在设备硬件设置中需要配置：

1. **通信方式**: 选择"网络"（而非"串口"）
2. **网络参数**:
   - 本机IP: 例如 `192.168.1.120`
   - 网络掩码: 例如 `255.255.255.0`
   - 默认网关: 例如 `192.168.1.1`
   - 端口号: 例如 `8989`
   - 自动获取IP: 根据实际情况选择开启或关闭
3. **脚本配置**: 启用相应的LUA脚本（如 `checkConnectivity.lua`、`getString_json.lua` 等）

### 3. PC端配置

确保PC与设备在同一网络，或通过网线直连。

## 使用方法

### 基本使用

```python
from embedded_board_tcp_client import EmbeddedBoardTCPClient

# 创建客户端（使用设备的IP和端口）
client = EmbeddedBoardTCPClient(host="192.168.1.19", port=8989)

# 连接到设备
if client.connect():
    # 发送数据
    client.send_json_format(1, "Hello World")
    
    # 断开连接
    client.disconnect()
```

### 使用上下文管理器

```python
from embedded_board_tcp_client import EmbeddedBoardTCPClient

with EmbeddedBoardTCPClient(host="192.168.1.19", port=8989) as client:
    if client.connected:
        # 发送数据
        client.send_json_format(1, "Test")
# 自动断开连接
```

### 数据格式

#### 1. 简易格式
```python
# 格式: xxx [Rn]xxx\r\n
client.send_simple_format(register_num=1, content="Hello")
```

#### 2. JSON格式（单寄存器）
```python
# 格式: {regNum=n,content=xxx}\r\n
client.send_json_format(register_num=1, content="123.45")  # 数字
client.send_json_format(register_num=2, content="Text")    # 文本（自动加引号）
```

#### 3. JSON格式（多寄存器）
```python
# 格式: {regNum=n,content=xxx}{regNum=n,content=xxx}...\r\n
client.send_json_multi_format([
    (1, "Data1"),
    (2, "Data2"),
    (3, "123")
])
```

#### 4. 特殊符号兼容格式
```python
# 格式: 01内容1 | 11内容2 | 12内容3\r\n
client.send_special_format([
    (1, "内容1!@#$%^"),
    (2, "内容2"),
    (3, "内容3")
])
```

### 远程指令控制

#### 打印控制（寄存器20）
```python
from embedded_board_tcp_client import PrintCommand

client.control_print(PrintCommand.START)      # 开启喷印
client.control_print(PrintCommand.PAUSE)      # 暂停喷印
client.control_print(PrintCommand.PRINT)       # 强制喷印
client.control_print(PrintCommand.CONTINUE)   # 继续喷印
client.control_print(PrintCommand.STOP)       # 退出喷印
client.control_print(PrintCommand.RESET_COUNT) # 重置计数
```

#### 信息获取（寄存器22）
```python
from embedded_board_tcp_client import InfoCommand

client.get_info(InfoCommand.INK_VOLUME)  # 查看墨量
client.get_info(InfoCommand.ERROR)      # 获取报警信息
```

#### 文件切换（寄存器24）
```python
client.switch_file("new_file")  # 文件名不含.msgx后缀
```

### 接收数据

```python
def on_receive(message: str):
    print(f"收到数据: {message}")
    # 解析响应，例如: inkVolume=85 或 error=xxx

client.set_receive_callback(on_receive)
```

## 运行示例

### 命令行运行

```bash
# 运行基本示例
python embedded_board_tcp_client.py basic

# 运行远程指令示例
python embedded_board_tcp_client.py commands

# 运行交互式示例
python embedded_board_tcp_client_example.py
```

### 交互式使用

运行 `embedded_board_tcp_client_example.py` 可以进入交互式菜单，方便测试各种功能。

## 协议说明

### 数据格式要求

1. **所有数据必须以 `\r\n` 结尾**
2. **寄存器号范围**:
   - **普通数据寄存器**: 1-16（用于存储普通数据）
   - **特殊功能寄存器**: 20, 22, 24
     - 寄存器20: 打印控制
     - 寄存器22: 信息获取
     - 寄存器24: 文件切换
3. **数据格式与寄存器号**:
   - JSON格式支持所有寄存器号（1-16, 20, 22, 24）
   - 简易格式主要用于普通数据寄存器（1-16）
   - 特殊符号兼容格式支持所有寄存器号

### 响应格式

- 信息查询响应: `key=value` 格式（如 `inkVolume=85`）
- 打印计数响应: 完成打印后回传 `\r\n` 结尾的打印计数数据

## 故障排查

### 连接失败 - "Connection refused" 错误

**症状：** 可以ping通设备，但TCP连接被拒绝（Connection refused）

**原因分析：**
- ✅ 网络层正常（ping成功）
- ❌ TCP端口未开放或服务未运行

**解决步骤：**

#### 1. 使用诊断工具
```bash
# 运行完整诊断
python diagnose_connection.py 192.168.1.100 8989

# 或快速检查端口
python quick_check_port.py 192.168.1.100 8989
```

#### 2. 检查设备配置（最重要！）

**在设备硬件设置界面中检查：**

a) **通信方式**
   - ✅ 必须选择 **"网络"**（而不是"串口"）
   - 如果选择了"串口"，TCP连接会被拒绝

b) **网络参数**
   - 本机IP: 确认是否为 `192.168.1.100`
   - 端口号: 确认是否为 `8989`（或你使用的其他端口）
   - 如果端口号不同，需要在代码中修改

c) **LUA脚本（关键！）**
   - ✅ 必须启用相应的LUA脚本
   - 找到"当前脚本"选项
   - 点击"切换"，选择脚本（如 `getString_json.lua`）
   - 点击"启用"或"加载"
   - ⚠️ **脚本未启用会导致TCP服务不启动**

#### 3. 检查端口是否开放
```bash
# 方法1: 使用telnet
telnet 192.168.1.100 8989

# 方法2: 使用nc (netcat)
nc -zv 192.168.1.100 8989

# 方法3: 使用Python脚本
python quick_check_port.py 192.168.1.100 8989
```

如果端口未开放，会看到：
- `Connection refused` - 端口未开放
- `Connection timeout` - 端口可能被防火墙阻止

#### 4. 扫描端口（如果默认端口不对）
```bash
# 运行诊断工具，选择扫描端口范围
python diagnose_connection.py 192.168.1.100 8989
# 然后选择 'y' 扫描端口范围
```

#### 5. 常见问题检查清单

- [ ] 设备硬件设置中选择了"网络"通信方式
- [ ] 设备端口号配置正确（与代码中的端口号一致）
- [ ] 已启用LUA脚本（如 `getString_json.lua`）
- [ ] 脚本状态显示为"启用"（不是"停用"）
- [ ] 设备已重启（某些设备需要重启才能生效）
- [ ] PC和设备在同一网段
- [ ] 防火墙未阻止连接

### 其他连接问题

1. **检查网络连接**
   - 确认PC和设备在同一网络
   - 使用 `ping` 命令测试设备IP是否可达
   - 检查防火墙设置

2. **检查设备配置**
   - 确认设备硬件设置中选择了"网络"通信方式
   - 确认IP地址、端口号配置正确
   - 确认已启用相应的LUA脚本

### 数据发送失败

1. 确认连接状态
2. 检查数据格式是否正确（必须以 `\r\n` 结尾）
3. 检查寄存器号是否在有效范围内

### 未收到响应

1. 确认设备已启用回传功能
2. 检查接收回调函数是否正确设置
3. 确认设备脚本配置正确

## 注意事项

1. **网络配置**: 确保PC和设备IP在同一网段
2. **端口号**: 默认端口8989，可根据设备配置修改
3. **数据格式**: 严格按照协议格式发送数据
4. **线程安全**: 客户端使用独立线程接收数据，发送和接收是线程安全的
5. **超时设置**: 连接和接收都有超时机制，可根据实际情况调整

## 参考文档

- 嵌入式主板串口通信说明文档（版本1.0.4）
- 设备硬件设置界面说明

