# 项目概览

## 发泡板区域化二维码标识系统

### 项目文件结构

```
foam_board_qrcode/
├── config/
│   └── qrcode_params.yaml          # ROS2参数配置文件
├── foam_board_qrcode/              # Python包主目录
│   ├── __init__.py                 # 包初始化
│   ├── board_qrcode_generator.py   # 核心生成器类
│   └── qrcode_generator_node.py    # ROS2节点
├── launch/
│   └── qrcode_generator.launch.py  # ROS2 launch文件
├── scripts/
│   └── simple_example.py           # 简单使用示例
├── test/
│   └── test_generate_qrcodes.py    # 完整测试脚本（生成示例二维码）
├── resource/
│   └── foam_board_qrcode           # ROS2资源文件
├── package.xml                     # ROS2包描述文件
├── setup.py                        # Python包安装脚本
├── requirements.txt                # Python依赖列表
├── README.md                       # 完整文档
├── QUICKSTART.md                   # 快速入门指南
└── PROJECT_OVERVIEW.md             # 本文件
```

### 核心组件

#### 1. FoamBoardQRCodeGenerator 类
**文件**: `foam_board_qrcode/board_qrcode_generator.py`

**主要功能**：
- 计算网格尺寸
- 生成板ID
- 创建区域数据
- 生成JSON编码
- 生成二维码图像
- 解码二维码数据
- 区域位置查询

**核心方法**：
```python
generate_complete_board_qrcode()  # 一站式生成
decode_board_qrcode()            # 解码JSON
get_region_code_by_position()    # 位置查询
get_region_grid()                # 获取2D网格
```

#### 2. ROS2节点
**文件**: `foam_board_qrcode/qrcode_generator_node.py`

**功能**：
- 订阅 `/board_parameters` 话题接收板参数
- 发布 `/qrcode_info` 话题输出二维码信息
- 发布 `/qrcode_image` 话题输出二维码图像（可选）
- 自动保存二维码到指定目录

**参数**：
- `grid_size_mm`: 网格尺寸
- `qr_size_mm`: 二维码尺寸
- `error_correction`: 错误纠正级别
- `output_directory`: 输出目录

### 数据格式

#### 输入：板参数
```json
{
  "board_id": "可选，板唯一标识",
  "width": 板宽度（mm），
  "length": 板长度（mm），
  "custom_data": [可选，自定义区域编码数组]
}
```

#### 输出：二维码JSON内容
```json
{
  "id": "板ID",
  "w": 列数,
  "h": 行数,
  "data": [区域编码数组]
}
```

#### 输出：生成结果信息
```json
{
  "board_id": "板ID",
  "width_mm": 板宽度,
  "length_mm": 板长度,
  "grid_cols": 列数,
  "grid_rows": 行数,
  "total_regions": 总区域数,
  "json_data": "JSON字符串",
  "json_size_bytes": JSON字节数,
  "output_file": "输出文件路径"
}
```

### 使用场景

#### 场景1：独立Python脚本
直接使用 `FoamBoardQRCodeGenerator` 类生成二维码。

**适用于**：
- 离线批量生成
- 测试和开发
- 简单的生产脚本

**示例**：`scripts/simple_example.py`

#### 场景2：ROS2系统集成
启动ROS2节点，通过话题通信生成二维码。

**适用于**：
- 与生产系统集成
- 实时生成需求
- 分布式系统

**启动方式**：
```bash
ros2 launch foam_board_qrcode qrcode_generator.launch.py
```

#### 场景3：作为库使用
在其他Python程序中导入使用。

**适用于**：
- 嵌入到现有系统
- 自定义处理流程
- 复杂业务逻辑

```python
from foam_board_qrcode.board_qrcode_generator import FoamBoardQRCodeGenerator
```

### 技术规格

| 项目 | 规格 |
|------|------|
| 板宽度 | 100-200 mm |
| 板长度 | 500 mm（可调） |
| 网格尺寸 | 50×50 mm（可调） |
| 二维码尺寸 | 20-30 mm（推荐25mm） |
| JSON大小 | 通常 90-200 字节 |
| 错误纠正 | L/M/Q/H 可选 |
| 生成速度 | <0.1秒/个 |

### 测试与验证

#### 运行测试
```bash
python3 test/test_generate_qrcodes.py
```

**测试覆盖**：
- 标准板（150×500mm）
- 小尺寸板（100×500mm）
- 大尺寸板（200×500mm）
- 自定义区域数据
- 批量生产（5个板）
- 真实生产场景模拟

#### 生成的文件
测试会在 `/tmp/` 目录生成11个示例二维码文件：
- `test_board_*.png` - 各种尺寸测试
- `batch_*.png` - 批量生产示例
- `test_production_real.png` - 真实场景模拟

### 依赖项

#### Python依赖
- `qrcode[pil]` >= 7.0 - 二维码生成
- `Pillow` >= 9.0 - 图像处理
- `numpy` >= 1.20 - 数组操作

#### ROS2依赖
- `rclpy` - ROS2 Python客户端
- `std_msgs` - 标准消息类型
- `sensor_msgs` - 传感器消息（图像）
- `cv_bridge` - 可选，图像转换

#### 系统要求
- Ubuntu 20.04/22.04
- ROS2 Humble/Iron/Rolling
- Python 3.8+

### 性能优化建议

1. **批量生产**：预生成板ID和区域数据，避免重复计算
2. **网络传输**：使用压缩传输JSON数据
3. **存储**：PNG格式已经很小（1-2KB），无需额外压缩
4. **打印质量**：推荐300dpi以上打印分辨率

### 扩展功能建议

1. **数据库集成**：记录板ID与生产信息的映射
2. **条码混合**：同时生成一维条码和二维码
3. **加密签名**：对JSON数据进行签名防篡改
4. **在线查询**：通过API查询板信息
5. **批次追溯**：建立完整的生产追溯链

### 维护与支持

**问题报告**：在项目仓库提交Issue

**功能建议**：欢迎提交Pull Request

**技术咨询**：联系维护团队

### 许可证

MIT License - 可自由使用、修改和分发

---

**版本**: v0.1.0  
**更新日期**: 2024-12-03  
**作者**: Yang - 杭州国辰机器人

