# 线扫相机触发配置指南 / Line Scan Camera Trigger Configuration Guide

## 概览 / Overview

本 ROS2 包支持三种触发模式 / This package supports three trigger modes:

1. **双触发模式（默认，推荐）** / **Dual Trigger Mode (DEFAULT, RECOMMENDED)** ⭐
2. **仅帧触发** / **Frame Trigger Only**
3. **仅行触发** / **Line Trigger Only**

> **注意**: 默认配置 (`camera_params.yaml`) 现在使用双触发模式！

## 硬件连线 / Hardware Connections

### 典型配置 / Typical Configuration

- **Line0**: Encoder B 信号 / Encoder B signal
- **Line1**: 帧触发信号 / Frame trigger signal
- **Line3**: Encoder A 信号 / Encoder A signal

## 三种模式详解 / Three Modes Explained

### 1. 双触发模式（同时使用帧触发和行触发）⭐ **[默认配置]**

**最适合的应用场景：**
- 传送带/移动平台上的线扫成像
- 需要同步帧开始和逐行扫描的场景

**工作原理：**
1. **帧触发（Line1）**：外部信号触发开始新一帧的采集
2. **行触发（Encoder）**：编码器输出控制每一行的采集时机

**使用方法（默认配置）：**
```bash
# 相机节点（使用默认双触发配置）
ros2 launch hk_line_camera hk_line_camera.launch.py

# 相机 + 拼接节点（推荐用于双触发模式）
ros2 launch hk_line_camera camera_with_stitching.launch.py
```

或显式指定配置文件：
```bash
ros2 launch hk_line_camera hk_line_camera.launch.py \
  config_file:=config/camera_params_dual_trigger.yaml
```

**配置参数：**
```yaml
frame_trigger_enabled: true   # 启用帧触发
frame_trigger_source: 1       # Line1 作为帧触发源

line_trigger_enabled: true    # 启用行触发
line_trigger_source: 6        # EncoderModuleOut 作为行触发源

encoder_source_a: 3           # Line3 连接 Encoder A
encoder_source_b: 0           # Line0 连接 Encoder B
```

### 2. 仅帧触发模式

**适合的应用场景：**
- 静态场景的触发式采集
- 不需要编码器同步的应用

**使用方法：**
```bash
ros2 launch hk_line_camera hk_line_camera.launch.py \
  config_file:=config/camera_params_frame_trigger.yaml
```

**配置参数：**
```yaml
frame_trigger_enabled: true   # 启用帧触发
frame_trigger_source: 1       # Line1 作为触发源

line_trigger_enabled: false   # 禁用行触发
```

### 3. 仅行触发模式

**适合的应用场景：**
- 连续扫描，由编码器控制行频率
- 不需要帧同步信号

**使用方法：**
```bash
# 需要修改 camera_params.yaml: 设置 frame_trigger_enabled: false
ros2 launch hk_line_camera hk_line_camera.launch.py
```

**配置参数：**
```yaml
frame_trigger_enabled: false  # 禁用帧触发

line_trigger_enabled: true    # 启用行触发
line_trigger_source: 6        # EncoderModuleOut

encoder_source_a: 3
encoder_source_b: 0
```

## 触发源参考表 / Trigger Source Reference

### Frame Trigger Source (帧触发源)
| Value | Source | 说明 |
|-------|--------|------|
| 0 | Line0 | GPIO Line 0 |
| 1 | Line1 | GPIO Line 1 ⭐ |
| 2 | Line2 | GPIO Line 2 |
| ... | ... | ... |

### Line Trigger Source (行触发源)
| Value | Source | 说明 |
|-------|--------|------|
| 0 | Line0 | GPIO Line 0 |
| 1 | Line1 | GPIO Line 1 |
| 6 | EncoderModuleOut | Encoder output ⭐ |

## 常见问题 / FAQ

### Q: 什么时候需要同时使用两种触发？
**A:** 当您需要：
1. 精确控制每帧的开始时刻（通过帧触发）
2. 同时根据物体移动速度同步采集每一行（通过编码器行触发）

这是传送带检测系统的标准配置。

### Q: 帧触发和行触发的信号频率要求？
**A:** 
- 帧触发：相对低频，每次需要开始新帧时触发一次
- 行触发：高频，由编码器产生，频率取决于移动速度和分辨率需求

### Q: 如何验证触发配置是否正确？
**A:** 启动节点后，查看日志输出：
```
[INFO] Setting frame trigger parameters (帧触发)...
[INFO] Frame TriggerSource set to Line1
[INFO] Setting line trigger parameters (行触发)...
[INFO] Line TriggerSource set to EncoderModuleOut (6)
```

## 参数快速参考 / Quick Parameter Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `frame_trigger_enabled` | false | 是否启用帧触发 |
| `frame_trigger_source` | 1 | 帧触发源（Line1） |
| `line_trigger_enabled` | true | 是否启用行触发 |
| `line_trigger_source` | 6 | 行触发源（EncoderModuleOut） |
| `encoder_source_a` | 3 | Encoder A 连接的 Line |
| `encoder_source_b` | 0 | Encoder B 连接的 Line |

## 配置文件位置 / Configuration Files

- `config/camera_params.yaml` - 默认配置（仅行触发）
- `config/camera_params_dual_trigger.yaml` - 双触发配置 ⭐
- `config/camera_params_frame_trigger.yaml` - 仅帧触发配置

## 技术参考 / Technical Reference

基于海康威视 MVS SDK 3.0.1 示例：
- `Samples/64/C++/LineScanCamera/ParametrizeCamera_LineScanIOSettings/`

触发选择器值：
- `TriggerSelector = 6`: FrameBurstStart (帧触发)
- `TriggerSelector = 9`: LineStart (行触发)

