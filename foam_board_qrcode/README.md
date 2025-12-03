# 发泡板区域化二维码标识系统

## 概述

本软件包实现了对发泡板（宽度100-200mm，长度500mm）的区域化信息标识功能。通过将整块板按照50×50mm网格切分，为每个区域分配三位数编码，并将所有信息统一封装在一个二维码中，实现高效喷印与快速识别。

## 核心特性

- **整板单码方案**：整块板只需喷印一个二维码（20-30mm），避免多码对位难题
- **区域化编码**：按50×50mm网格切分，每个区域对应唯一三位数编码
- **精简数据格式**：采用JSON格式封装，通常 < 150字节
- **灵活配置**：支持自定义板尺寸、网格大小、区域编码
- **ROS2集成**：提供ROS2节点，可与生产系统无缝对接
- **批量生产**：支持批量板号管理和追溯

## 系统原理

### 数据封装格式

二维码包含以下信息（JSON格式）：

```json
{
  "id": "板ID（如：20241203-0012）",
  "w": 列数,
  "h": 行数,
  "data": [区域编码数组，行优先顺序]
}
```

### 示例

150mm × 500mm的板（3列 × 10行 = 30个区域）：

```json
{
  "id":"20241203-0012",
  "w":3,
  "h":10,
  "data":[101,102,103,201,202,203,301,302,303,401,402,403,501,502,503,601,602,603,701,702,703,801,802,803,901,902,903,999,999,999]
}
```

数据量：约120字节，可被标准二维码轻松容纳。

## 安装

### 依赖项

**系统依赖：**
```bash
sudo apt update
sudo apt install python3-pip python3-pil
```

**Python依赖：**
```bash
pip3 install qrcode[pil] numpy
```

**ROS2依赖：**
```bash
sudo apt install ros-<your-distro>-cv-bridge  # 可选，用于图像发布
```

### 构建

```bash
cd ~/gbx_line_scan
colcon build --packages-select foam_board_qrcode
source install/setup.bash
```

## 使用方法

### 1. 测试脚本（生成示例二维码）

运行测试脚本生成各种场景的示例二维码：

```bash
cd ~/gbx_line_scan/src/foam_board_qrcode
python3 test/test_generate_qrcodes.py
```

这将在 `/tmp/` 目录下生成多个测试二维码图片，包括：
- 标准板（150×500mm）
- 小尺寸板（100×500mm）
- 大尺寸板（200×500mm）
- 自定义区域数据
- 批量生产示例
- 真实生产场景模拟

### 2. ROS2节点方式

#### 启动节点

```bash
ros2 launch foam_board_qrcode qrcode_generator.launch.py
```

或指定输出目录：

```bash
ros2 launch foam_board_qrcode qrcode_generator.launch.py output_dir:=/home/user/qrcodes
```

#### 发送板参数生成二维码

**标准板（使用默认参数）：**
```bash
ros2 topic pub --once /board_parameters std_msgs/msg/String \
  '{data: "{\"width\":150.0,\"length\":500.0}"}'
```

**指定板ID：**
```bash
ros2 topic pub --once /board_parameters std_msgs/msg/String \
  '{data: "{\"board_id\":\"BOARD-001\",\"width\":150.0,\"length\":500.0}"}'
```

**自定义区域数据：**
```bash
ros2 topic pub --once /board_parameters std_msgs/msg/String \
  '{data: "{\"board_id\":\"CUSTOM-001\",\"width\":150.0,\"length\":500.0,\"custom_data\":[101,102,103,201,202,203,301,302,303,401,402,403,501,502,503,601,602,603,701,702,703,801,802,803,901,902,903,999,999,999]}"}'
```

#### 查看生成结果

```bash
# 查看二维码信息
ros2 topic echo /qrcode_info

# 查看生成的图像（如果安装了cv_bridge）
ros2 topic echo /qrcode_image
```

### 3. Python API方式

在你的Python程序中直接使用：

```python
from foam_board_qrcode.board_qrcode_generator import FoamBoardQRCodeGenerator

# 创建生成器
generator = FoamBoardQRCodeGenerator(grid_size=50.0)

# 生成二维码
result = generator.generate_complete_board_qrcode(
    board_width=150.0,
    board_length=500.0,
    board_id="BOARD-001",
    output_file="output.png",
    qr_size_mm=25
)

print(f"板ID: {result['board_id']}")
print(f"JSON大小: {result['json_size_bytes']} 字节")
print(f"总区域数: {result['total_regions']}")

# 解码二维码
board_data = generator.decode_board_qrcode(result['json_data'])

# 查询特定位置的区域编码
code = generator.get_region_code_by_position(board_data, col=0, row=0)
print(f"位置[0,0]的编码: {code}")

# 获取完整网格
grid = generator.get_region_grid(board_data)
print(grid)
```

## 配置参数

配置文件：`config/qrcode_params.yaml`

```yaml
qrcode_generator_node:
  ros__parameters:
    grid_size_mm: 50.0           # 网格尺寸（mm）
    qr_size_mm: 25               # 二维码尺寸（mm）
    error_correction: 'M'        # 错误纠正级别：'L'/'M'/'Q'/'H'
    output_directory: '/tmp/foam_board_qrcodes'
    default_board_width: 150.0   # 默认板宽（mm）
    default_board_length: 500.0  # 默认板长（mm）
```

### 错误纠正级别说明

- **L (Low)**: 7% 容错，适合理想环境
- **M (Medium)**: 15% 容错，推荐一般使用
- **Q (Quartile)**: 25% 容错，推荐生产环境
- **H (High)**: 30% 容错，最高容错但二维码更大

## 应用场景

### 1. 质量追溯

为每个区域记录质量等级：

```python
quality_zones = [
    101, 102, 103,  # 第1行：A级
    201, 202, 203,  # 第2行：A级
    401, 402, 403,  # 第3-4行：B级
    501, 502, 503,
    ...
]
```

### 2. 生产批次管理

```python
board_id = f"20241203-BATCH-A-{serial_number:04d}"
```

### 3. 区域功能标识

为不同区域标记不同用途或属性。

## 下游使用

下游设备扫描二维码后，可以：

1. **解析板信息**
   ```python
   board_data = generator.decode_board_qrcode(scanned_json)
   print(f"板ID: {board_data['id']}")
   print(f"网格: {board_data['w']}列 × {board_data['h']}行")
   ```

2. **定位区域**
   ```python
   # 根据物理位置计算区域
   col = int(x_mm / 50)
   row = int(y_mm / 50)
   code = generator.get_region_code_by_position(board_data, col, row)
   ```

3. **可视化网格**
   ```python
   grid = generator.get_region_grid(board_data)
   # 显示或处理2D数组
   ```

## 与喷码设备集成

### TIJ喷码机集成示例

```python
# 1. 生成二维码数据
result = generator.generate_complete_board_qrcode(...)

# 2. 将JSON数据发送到喷码机
# 根据你的喷码机接口协议，可能是：
# - 串口通信
# - TCP/IP
# - 文件传输
# - Modbus等

# 示例：串口发送
import serial
ser = serial.Serial('/dev/ttyUSB0', 115200)
command = f"PRINT_QR:{result['json_data']}\n"
ser.write(command.encode())

# 3. 喷码机接收后渲染二维码并喷印
```

## 性能指标

| 板尺寸 | 网格数 | JSON大小 | 二维码版本 | 生成时间 |
|--------|--------|----------|------------|----------|
| 100×500mm | 2×10=20 | ~90字节 | 2-3 | <0.1s |
| 150×500mm | 3×10=30 | ~120字节 | 3-4 | <0.1s |
| 200×500mm | 4×10=40 | ~150字节 | 4-5 | <0.1s |

## 故障排查

### 问题：无法导入qrcode模块

```bash
pip3 install qrcode[pil]
```

### 问题：生成的二维码无法扫描

1. 检查二维码尺寸是否足够（建议≥20mm）
2. 提高错误纠正级别（'Q'或'H'）
3. 确保打印/喷印质量清晰

### 问题：JSON数据过大

1. 减少网格数量（增大grid_size）
2. 使用更短的板ID
3. 简化区域编码

## API参考

### FoamBoardQRCodeGenerator

**主要方法：**

- `generate_complete_board_qrcode()`: 一站式生成板二维码
- `decode_board_qrcode()`: 解码二维码JSON数据
- `get_region_code_by_position()`: 根据位置查询区域编码
- `get_region_grid()`: 获取2D网格数组
- `calculate_grid_dimensions()`: 计算网格尺寸
- `generate_board_id()`: 生成唯一板ID

详细API文档请参考代码注释。

## 示例图片

运行测试脚本后，可以在 `/tmp/` 目录查看生成的二维码：

```bash
ls -lh /tmp/test_*.png /tmp/batch_*.png
```

使用手机或二维码扫描器查看内容，验证编码正确性。

## 技术支持

- **ROS2版本**: Humble/Iron/Rolling
- **Python版本**: ≥3.8
- **操作系统**: Ubuntu 20.04/22.04

## 许可证

MIT License

## 作者

Yang - 杭州国辰机器人

## 更新日志

### v0.1.0 (2024-12-03)
- 初始版本
- 实现基本二维码生成功能
- ROS2节点集成
- 测试脚本和文档

