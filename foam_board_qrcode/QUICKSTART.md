# 快速入门指南

## 5分钟上手

### 1. 安装依赖

```bash
cd ~/gbx_line_scan/src/foam_board_qrcode
pip3 install -r requirements.txt --user
```

### 2. 生成第一个二维码

**方法A：运行简单示例**
```bash
python3 scripts/simple_example.py
```

生成的二维码在：`/tmp/simple_example_qrcode.png`

**方法B：运行完整测试**
```bash
python3 test/test_generate_qrcodes.py
```

这将生成11个示例二维码，展示各种场景。

### 3. 在你的代码中使用

```python
from foam_board_qrcode.board_qrcode_generator import FoamBoardQRCodeGenerator

# 创建生成器
generator = FoamBoardQRCodeGenerator()

# 生成二维码
result = generator.generate_complete_board_qrcode(
    board_width=150.0,       # mm
    board_length=500.0,      # mm
    board_id="YOUR-ID-001",
    output_file="output.png"
)

print(f"二维码已生成: {result['output_file']}")
print(f"JSON大小: {result['json_size_bytes']} 字节")
```

### 4. ROS2集成（可选）

```bash
# 构建包
cd ~/gbx_line_scan
colcon build --packages-select foam_board_qrcode
source install/setup.bash

# 启动节点
ros2 launch foam_board_qrcode qrcode_generator.launch.py

# 发送板参数
ros2 topic pub --once /board_parameters std_msgs/msg/String \
  '{data: "{\"width\":150.0,\"length\":500.0}"}'
```

## 常见用例

### 场景1：生产线批量生成

```python
generator = FoamBoardQRCodeGenerator()

for i in range(100):
    board_id = f"BATCH-20241203-{i:04d}"
    generator.generate_complete_board_qrcode(
        board_width=150.0,
        board_length=500.0,
        board_id=board_id,
        output_file=f"/production/qrcodes/{board_id}.png"
    )
```

### 场景2：质量分区标识

```python
# 定义质量等级
quality_data = [
    101, 102, 103,  # 第1行：A级
    201, 202, 203,  # 第2行：A级
    301, 302, 303,  # 第3行：B级
    # ... 更多行
]

generator.generate_complete_board_qrcode(
    board_width=150.0,
    board_length=500.0,
    board_id="QA-001",
    custom_data=quality_data,
    output_file="qa_board.png"
)
```

### 场景3：下游扫描解码

```python
# 下游设备扫描二维码后
scanned_json = "从二维码扫描得到的JSON字符串"

# 解码
board_data = generator.decode_board_qrcode(scanned_json)

# 获取信息
print(f"板ID: {board_data['id']}")
print(f"网格: {board_data['w']} x {board_data['h']}")

# 根据物理位置查询区域编码
x_mm = 75.0  # 物理位置X坐标
y_mm = 125.0 # 物理位置Y坐标

col = int(x_mm / 50)  # 计算列
row = int(y_mm / 50)  # 计算行

region_code = generator.get_region_code_by_position(board_data, col, row)
print(f"位置({x_mm}, {y_mm})的区域编码: {region_code}")
```

## 查看生成的二维码

### 手机扫描
用手机扫描二维码应用扫描生成的PNG图片，即可看到JSON内容。

### 命令行解码（需要安装zbar）
```bash
sudo apt install zbar-tools
zbarimg /tmp/simple_example_qrcode.png
```

### 在线解码
上传PNG文件到任何在线二维码解码器，如：
- https://zxing.org/w/decode
- https://www.qrcode-monkey.com/

## 参数调优

### 调整网格大小
```python
# 使用100mm网格（适合大区域）
generator = FoamBoardQRCodeGenerator(grid_size=100.0)
```

### 增加容错级别
```python
# 使用高容错级别（适合恶劣环境）
generator.generate_qrcode_image(json_data, error_correction='Q')
```

### 调整二维码尺寸
```python
# 生成更大的二维码（30mm）
generator.generate_complete_board_qrcode(
    ...,
    qr_size_mm=30
)
```

## 故障排查

**问题：二维码太小无法扫描**
- 增加 `qr_size_mm` 参数至 25-30mm
- 提高打印/喷印分辨率

**问题：JSON数据超过二维码容量**
- 减少区域数量（增大grid_size）
- 使用更短的board_id
- 简化区域编码数值

**问题：扫描后数据损坏**
- 提高错误纠正级别至 'Q' 或 'H'
- 确保喷印质量清晰
- 避免二维码变形

## 下一步

- 阅读完整文档：[README.md](README.md)
- 查看API文档：源码中的详细注释
- 集成到生产系统：参考ROS2节点示例

