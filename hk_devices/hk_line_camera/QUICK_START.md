# 快速开始 - 双触发模式 + 图像拼接

## 默认配置说明

**默认配置现已设置为双触发模式**，适合大多数线扫相机应用场景：
- ✅ 帧触发（Line1）：控制何时开始新的一帧
- ✅ 行触发（Encoder）：控制每一行的采集时机
- ✅ 图像拼接：自动将多行图像拼接成完整图像

## 快速启动

### 方式一：相机 + 拼接（推荐）

直接启动相机和拼接节点：

```bash
cd /home/yang/gbx_line_scan
source install/setup.bash
ros2 launch hk_line_camera camera_with_stitching.launch.py
```

这将启动：
- 相机节点（双触发模式）
- 图像拼接节点

### 方式二：仅相机节点

如果只需要相机节点（不需要拼接）：

```bash
ros2 launch hk_line_camera hk_line_camera.launch.py
```

## 查看图像

### 查看原始线扫图像

```bash
ros2 run rqt_image_view rqt_image_view /image_raw
```

### 查看拼接后的图像

```bash
ros2 run rqt_image_view rqt_image_view /image_stitched
```

或使用提供的脚本：

```bash
ros2 run hk_line_camera view_stitched_image.py
```

## 硬件连线要求

确保以下连线正确：

| 相机接口 | 连接信号 | 说明 |
|---------|---------|------|
| **Line0** | Encoder B | 编码器 B 相信号 |
| **Line1** | Frame Trigger | 帧触发信号（外部触发源） |
| **Line3** | Encoder A | 编码器 A 相信号 |

## 参数调整

### 修改曝光时间

编辑 `config/camera_params.yaml`：

```yaml
exposure_time_us: 300.0  # 单位：微秒
```

### 修改拼接参数

编辑 `config/stitching_params.yaml`：

```yaml
max_height: 0              # 最大拼接高度（0=无限制）
max_stitch_count: 0        # 最大拼接帧数（0=无限制）
reset_on_max_height: true  # 达到最大高度时重置
publish_rate: 10.0         # 发布频率（Hz）
```

## 临时切换到其他触发模式

### 仅帧触发模式

```bash
ros2 launch hk_line_camera hk_line_camera.launch.py \
  config_file:=config/camera_params_frame_trigger.yaml
```

### 禁用帧触发（仅行触发）

临时修改 `camera_params.yaml`：
```yaml
frame_trigger_enabled: false  # 禁用帧触发
line_trigger_enabled: true    # 保持行触发
```

## 验证配置

启动后检查终端输出，应该看到：

```
[INFO] Setting frame trigger parameters (帧触发)...
[INFO] Frame TriggerSource set to Line1
[INFO] Setting line trigger parameters (行触发)...
[INFO] Line TriggerSource set to EncoderModuleOut (6)
[INFO] Camera started grabbing
```

## 常见问题

### Q: 没有图像输出？
**A**: 检查：
1. 帧触发信号是否正常（Line1 有信号输入）
2. 编码器是否正常工作（Line0 和 Line3 有信号）
3. 使用 `ros2 topic list` 确认话题是否存在
4. 使用 `ros2 topic echo /image_raw --no-arr` 查看图像元数据

### Q: 图像拼接效果不好？
**A**: 调整拼接参数：
- 如果图像太大，设置 `max_height` 限制高度
- 如果拼接速度慢，调整 `publish_rate`
- 检查编码器信号是否稳定

### Q: 如何切换回单触发模式？
**A**: 修改 `config/camera_params.yaml`：
```yaml
frame_trigger_enabled: false  # 禁用帧触发
# 或
line_trigger_enabled: false   # 禁用行触发
```

## 更多信息

- 详细触发配置说明：查看 `TRIGGER_GUIDE.md`
- 完整功能介绍：查看 `README.md`
- SDK 示例：`src/SDK/MVS-3.0.1_x86_64_20241128/Samples/`

## 技术支持

配置基于：
- 海康威视 MVS SDK 3.0.1
- C++ 示例：`LineScanCamera/ParametrizeCamera_LineScanIOSettings`

