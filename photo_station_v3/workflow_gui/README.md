# Workflow GUI - Photo Station V3

完整的工作流控制 GUI，集成电机控制、光源控制和双相机图像显示。

## 功能特点

### 电机控制
- 双电机独立控制（前电机/后电机）
- 速度设置（-3000 ~ 3000 RPM）
- 启用/禁用控制
- 同步控制（同时启用/禁用/设速）
- 紧急停止按钮
- 实时速度反馈显示

### 光源控制
- 双光源独立控制（光源1/光源2）
- 亮度调节（0-255）
- 开关控制
- 实时状态显示（电压、电流、功率）
- 连接状态指示

### 图像显示
- 双相机拼接图像实时显示
- 前相机: `/camera_front/front/image_stitched`
- 后相机: `/camera_rear/rear/image_stitched`
- 图像保存功能
- 拼接重置功能
- 帧计数和尺寸显示

## 安装

### 依赖项
```bash
# ROS2 依赖
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs

# Python 依赖
pip3 install PyQt5 opencv-python numpy
```

### 编译
```bash
cd ~/gbx_line_scan
colcon build --packages-select workflow_gui
source install/setup.bash
```

## 使用方法

### 完整启动（所有节点 + GUI）
```bash
ros2 launch workflow_gui workflow_gui.launch.py
```

### 仅启动 GUI（其他节点已运行）
```bash
ros2 launch workflow_gui gui_only.launch.py
```

### 启动参数
```bash
# 仅启动 GUI
ros2 launch workflow_gui workflow_gui.launch.py gui_only:=true

# 跳过相机启动
ros2 launch workflow_gui workflow_gui.launch.py skip_cameras:=true

# 跳过电机启动
ros2 launch workflow_gui workflow_gui.launch.py skip_motors:=true

# 跳过光源控制器启动
ros2 launch workflow_gui workflow_gui.launch.py skip_lights:=true

# 自定义设备路径
ros2 launch workflow_gui workflow_gui.launch.py motor_device:=/dev/ttyUSB0 light_ip:=192.168.3.100
```

## ROS2 接口

### 电机控制 Topics

#### 发布
| Topic | 类型 | 描述 |
|-------|------|------|
| `/predictor/front_motor/cmd_vel` | `Float64` | 前电机目标速度 (RPM) |
| `/predictor/rear_motor/cmd_vel` | `Float64` | 后电机目标速度 (RPM) |
| `/predictor/front_motor/enable` | `Bool` | 前电机启用/禁用 |
| `/predictor/rear_motor/enable` | `Bool` | 后电机启用/禁用 |

#### 订阅
| Topic | 类型 | 描述 |
|-------|------|------|
| `/predictor/front_motor/velocity` | `Float64` | 前电机当前速度 (RPM) |
| `/predictor/rear_motor/velocity` | `Float64` | 后电机当前速度 (RPM) |

### 光源控制 Topics

#### 发布
| Topic | 类型 | 描述 |
|-------|------|------|
| `light1/control` | `Bool` | 光源1开关 |
| `light2/control` | `Bool` | 光源2开关 |
| `light1/set_brightness` | `Int32` | 光源1亮度 (0-255) |
| `light2/set_brightness` | `Int32` | 光源2亮度 (0-255) |

#### 订阅
| Topic | 类型 | 描述 |
|-------|------|------|
| `light1/status` | `Bool` | 光源1状态 |
| `light2/status` | `Bool` | 光源2状态 |
| `light1/brightness` | `Int32` | 光源1当前亮度 |
| `light2/brightness` | `Int32` | 光源2当前亮度 |
| `light1/voltage` | `Float32` | 光源1电压 (V) |
| `light1/current` | `Float32` | 光源1电流 (A) |
| `light2/voltage` | `Float32` | 光源2电压 (V) |
| `light2/current` | `Float32` | 光源2电流 (A) |
| `connection_status` | `Bool` | 光源控制器连接状态 |

### 图像 Topics

#### 订阅
| Topic | 类型 | 描述 |
|-------|------|------|
| `/camera_front/front/image_stitched` | `Image` | 前相机拼接图像 |
| `/camera_rear/rear/image_stitched` | `Image` | 后相机拼接图像 |

### Services

| Service | 类型 | 描述 |
|---------|------|------|
| `/camera_front/image_stitching_front/reset_stitching` | `Trigger` | 重置前相机拼接 |
| `/camera_rear/image_stitching_rear/reset_stitching` | `Trigger` | 重置后相机拼接 |

## 界面说明

### 配色方案
采用深色主题（黑/白/灰），与 image_viewer 保持一致：
- 背景色: `#1a1a1a`
- 面板背景: `#2a2a2a`
- 边框: `#444444`
- 文字: `#e0e0e0`
- 强调色: 绿色（开启状态）/ 红色（紧急停止）

### 布局
```
┌─────────────────────────────────────────────────────────────┐
│                    Photo Station V3 - 完整工作流控制系统         │
├─────────────────────────────┬───────────────────────────────┤
│                             │                               │
│     ┌─────────────────┐     │    ┌───────────────────────┐  │
│     │   电机控制       │     │    │   前相机图像           │  │
│     │  ┌─────┬─────┐  │     │    │                       │  │
│     │  │前电机│后电机│  │     │    │                       │  │
│     │  └─────┴─────┘  │     │    └───────────────────────┘  │
│     │  同步控制 紧急停止│     │                               │
│     └─────────────────┘     │    ┌───────────────────────┐  │
│                             │    │   后相机图像           │  │
│     ┌─────────────────┐     │    │                       │  │
│     │   光源控制       │     │    │                       │  │
│     │  ┌─────┬─────┐  │     │    └───────────────────────┘  │
│     │  │光源1│光源2│  │     │                               │
│     │  └─────┴─────┘  │     │                               │
│     └─────────────────┘     │                               │
│                             │                               │
├─────────────────────────────┴───────────────────────────────┤
│ 状态栏                                                       │
└─────────────────────────────────────────────────────────────┘
```

## 文件结构

```
workflow_gui/
├── launch/
│   ├── workflow_gui.launch.py    # 完整启动文件
│   └── gui_only.launch.py        # 仅GUI启动文件
├── scripts/
│   ├── __init__.py
│   └── workflow_gui.py           # 主GUI程序
├── resource/
│   └── workflow_gui              # ament资源标记文件
├── package.xml                   # ROS2包描述
├── setup.py                      # Python包配置
├── setup.cfg                     # 安装配置
└── README.md                     # 本文档
```

## 故障排除

### GUI 无法显示图像
1. 检查相机节点是否运行: `ros2 topic list | grep image`
2. 检查图像话题是否有数据: `ros2 topic hz /camera_front/front/image_stitched`

### 电机无响应
1. 检查串口设备: `ls -la /dev/ttyACM*`
2. 检查 predictor_node 是否运行: `ros2 node list | grep predictor`

### 光源控制器未连接
1. 检查网络连接: `ping 192.168.3.101`
2. 检查 hk_light_controller 节点: `ros2 node list | grep light`

## 许可证

Apache-2.0
