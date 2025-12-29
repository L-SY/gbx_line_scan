# HK Light Controller ROS2 Package

ROS2 package for controlling Hikvision light controllers using the MVS SDK.

## Features

- Control two independent light channels
- Set brightness for each light
- Configure trigger source
- Publish status information via ROS2 topics
- Control via ROS2 topics and services

## Topics

### Published Topics

- `light1/status` (std_msgs/Bool) - Status of light 1 (ON/OFF)
- `light2/status` (std_msgs/Bool) - Status of light 2 (ON/OFF)
- `light1/brightness` (std_msgs/Int32) - Current brightness of light 1
- `light2/brightness` (std_msgs/Int32) - Current brightness of light 2
- `trigger_source` (std_msgs/String) - Current trigger source
- `connection_status` (std_msgs/Bool) - Connection status to the controller

### Subscribed Topics

- `light1/control` (std_msgs/Bool) - Control light 1 ON/OFF
- `light2/control` (std_msgs/Bool) - Control light 2 ON/OFF
- `light1/set_brightness` (std_msgs/Int32) - Set brightness for light 1
- `light2/set_brightness` (std_msgs/Int32) - Set brightness for light 2
- `set_trigger_source` (std_msgs/String) - Set trigger source (Software, Line0, Line1, Line2, Line3)

### Services

- `light1/set_enabled` (std_srvs/SetBool) - Enable/disable light 1
- `light2/set_enabled` (std_srvs/SetBool) - Enable/disable light 2
- `get_status` (std_srvs/Trigger) - Get current status of all lights

## Parameters

- `ip_address` (string, default: "192.168.1.100") - IP address of the light controller
- `status_publish_rate` (double, default: 1.0) - Rate at which status is published (Hz)

## Usage

### 启动节点

```bash
cd ~/gbx_line_scan
source install/setup.bash
ros2 launch hk_light_controller hk_light_controller.launch.py
```

或者指定 IP 地址：

```bash
ros2 launch hk_light_controller hk_light_controller.launch.py ip_address:=192.168.1.100
```

### 使用话题控制光源

#### 控制光源开启/关闭

```bash
# 开启光源1
ros2 topic pub /light1/control std_msgs/msg/Bool "{data: true}" --once

# 关闭光源1
ros2 topic pub /light1/control std_msgs/msg/Bool "{data: false}" --once

# 开启光源2
ros2 topic pub /light2/control std_msgs/msg/Bool "{data: true}" --once

# 关闭光源2
ros2 topic pub /light2/control std_msgs/msg/Bool "{data: false}" --once
```

#### 设置亮度

```bash
# 设置光源1亮度（值取决于硬件，通常 0-255 或 0-100）
ros2 topic pub /light1/set_brightness std_msgs/msg/Int32 "{data: 100}" --once

# 设置光源2亮度
ros2 topic pub /light2/set_brightness std_msgs/msg/Int32 "{data: 100}" --once
```

#### 设置触发源

```bash
# 软件触发
ros2 topic pub /set_trigger_source std_msgs/msg/String "{data: 'Software'}" --once

# Line0 触发
ros2 topic pub /set_trigger_source std_msgs/msg/String "{data: 'Line0'}" --once

# Line1 触发
ros2 topic pub /set_trigger_source std_msgs/msg/String "{data: 'Line1'}" --once
```

### 使用服务控制光源

```bash
# 开启光源1
ros2 service call /light1/set_enabled std_srvs/srv/SetBool "{data: true}"

# 关闭光源2
ros2 service call /light2/set_enabled std_srvs/srv/SetBool "{data: false}"

# 获取状态
ros2 service call /get_status std_srvs/srv/Trigger
```

### 查看状态

```bash
# 查看连接状态
ros2 topic echo /connection_status

# 查看光源1状态
ros2 topic echo /light1/status

# 查看光源2状态
ros2 topic echo /light2/status

# 查看亮度
ros2 topic echo /light1/brightness
ros2 topic echo /light2/brightness

# 查看触发源
ros2 topic echo /trigger_source
```

### 使用控制脚本

```bash
# 运行示例控制脚本
cd ~/gbx_line_scan/src/hk_devices/hk_light_controller/scripts
./control_lights.sh
```

### 使用 Qt GUI 界面

提供了一个图形界面来测试和控制光源：

```bash
# 方法1: 使用启动脚本（推荐）
cd ~/gbx_line_scan/src/hk_devices/hk_light_controller/scripts
./run_gui.sh

# 方法2: 手动启动
# 终端1: 启动光源控制器节点
ros2 launch hk_light_controller hk_light_controller.launch.py

# 终端2: 启动 GUI
ros2 run hk_light_controller light_controller_gui
```

GUI 界面功能：
- ✅ 显示连接状态
- ✅ 控制光源1和光源2的开启/关闭
- ✅ 调整光源1和光源2的亮度（0-255）
- ✅ 设置触发源（Software, Line0, Line1, Line2, Line3）
- ✅ 实时显示状态信息
- ✅ 刷新状态按钮

## Building

```bash
cd ~/gbx_line_scan
colcon build --packages-select hk_light_controller
source install/setup.bash
```

## Environment Setup

Before running the node, you may need to set up the MVS SDK environment variables:

```bash
# Set SDK path (adjust path as needed)
export MVCAM_SDK_PATH=/home/yang/gbx_line_scan/src/SDK/MVS-4.6.1_x86_64_20251217/MVS
export MVCAM_COMMON_RUNENV=${MVCAM_SDK_PATH}/lib
export LD_LIBRARY_PATH=${MVCAM_COMMON_RUNENV}/64:${LD_LIBRARY_PATH}

# Or source the SDK setup script
source /home/yang/gbx_line_scan/src/SDK/MVS-4.6.1_x86_64_20251217/set_env_path.sh /opt/MVS
```

## Troubleshooting

### "No light controller interfaces found!" 错误说明

这个错误表示：
- ✅ SDK 初始化成功（否则会有不同的错误信息）
- ❌ 但是枚举光源控制器接口时返回了 0 个接口

**这意味着程序运行正常，但找不到光源控制器设备。**

### 诊断步骤

#### 1. 运行诊断脚本（推荐）

首先运行综合诊断脚本来检查所有可能的问题：

```bash
cd ~/gbx_line_scan/src/hk_devices/hk_light_controller/scripts
./diagnose_connection.sh
```

这个脚本会检查：
- SDK 安装和路径
- 环境变量设置
- 库文件存在性
- 网络连接（GigE 设备）
- USB 设备（USB 设备）
- 驱动加载状态
- 官方 MVS 软件是否在运行（可能会阻止 SDK 访问）

#### 2. 运行简单测试脚本

如果诊断脚本显示一切正常，运行连接测试：

```bash
cd ~/gbx_line_scan/src/hk_devices/hk_light_controller/scripts
./test_sdk_connection.sh
```

这个脚本会：
- 检查 SDK 环境变量
- 测试 SDK 初始化
- 枚举光源控制器接口
- 显示找到的接口信息

#### 3. 运行 SDK 官方示例程序

如果测试脚本也找不到设备，尝试运行 SDK 自带的示例程序：

```bash
# 设置环境变量
export MVCAM_SDK_PATH=/home/yang/gbx_line_scan/src/SDK/MVS-4.6.1_x86_64_20251217/MVS
export MVCAM_COMMON_RUNENV=${MVCAM_SDK_PATH}/lib
export LD_LIBRARY_PATH=${MVCAM_COMMON_RUNENV}/64:${LD_LIBRARY_PATH}

# 编译并运行示例程序
cd ~/gbx_line_scan/src/SDK/MVS-4.6.1_x86_64_20251217/MVS/Samples/64/C++/LightController/ConfigLightController
g++ -g -o ConfigLightController ConfigLightController.cpp \
    -I../../../../../include \
    -Wl,-rpath=${MVCAM_COMMON_RUNENV}/64 \
    -L${MVCAM_COMMON_RUNENV}/64 \
    -lMvCameraControl -lpthread

./ConfigLightController
```

如果官方示例程序也找不到设备，说明问题不在 ROS2 节点，而是：
- 设备未连接
- 设备未上电
- 网络连接问题（GigE 设备）
- SDK 驱动未正确加载

#### 4. 检查设备连接

**对于 GigE（网络）设备：**
```bash
# 检查网络连接
ping 192.168.1.100

# 检查网络接口
ip addr show

# 检查路由
ip route
```

**对于 USB 设备：**
```bash
# 检查 USB 设备
lsusb | grep -i hikvision

# 检查驱动是否加载
lsmod | grep mvusbdrv

# 如果需要，加载驱动
sudo modprobe mvusbdrv
```

#### 5. 重要提示：关闭官方 MVS 软件

**如果官方 MVS 软件正在运行，SDK 可能无法访问设备！**

```bash
# 检查是否有 MVS 进程在运行
ps aux | grep MVS

# 如果有，关闭它
pkill -f MVS
```

#### 6. 检查 SDK 环境

确保 SDK 环境变量正确设置：

```bash
echo $MVCAM_SDK_PATH
echo $MVCAM_COMMON_RUNENV
echo $LD_LIBRARY_PATH | grep MVS
```

如果环境变量未设置，运行：
```bash
source src/hk_devices/hk_light_controller/scripts/setup_sdk_env.sh
```

### 其他常见问题

This error means the SDK cannot find any light controller devices. Check:

1. **Physical Connection**: Ensure the light controller is connected and powered on
2. **Network Connection**: If using GigE, ensure network connectivity to the controller's IP address
3. **SDK Environment**: Verify SDK environment variables are set:
   ```bash
   echo $MVCAM_SDK_PATH
   echo $LD_LIBRARY_PATH
   ```
4. **SDK Libraries**: Ensure SDK libraries are accessible:
   ```bash
   ls -la $MVCAM_SDK_PATH/lib/64/libMvCameraControl.so
   ```
5. **Permissions**: Some operations may require root privileges for USB devices

### "Initialize SDK failed!"

- Check that SDK libraries are in the library path
- Verify SDK version compatibility
- Check system logs for driver issues

### Node starts but cannot control lights

- Check that `interface_initialized_` is true (check connection_status topic)
- Verify the light controller is responding
- Check network connectivity if using GigE

### 程序运行但显示错误信息

**这是正常的！** 程序现在设计为即使找不到设备也不会崩溃。它会：
- 继续运行
- 发布连接状态（`connection_status` topic 会显示 `false`）
- 所有控制命令会被忽略并显示警告

你可以通过以下方式检查状态：
```bash
# 检查连接状态
ros2 topic echo /connection_status

# 如果显示 data: false，说明设备未连接
```

一旦设备连接成功，程序会自动检测到并开始工作。

## Notes

- The package requires the MVS SDK to be available at `../../SDK/MVS-4.6.1_x86_64_20251217/MVS`
- The actual parameter names for controlling lights may vary depending on the specific light controller model
- Some parameters might need adjustment based on your hardware configuration

