#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本：生成各种场景的发泡板二维码示例
"""

import sys
import os
from pathlib import Path

# 添加包路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from foam_board_qrcode.board_qrcode_generator import FoamBoardQRCodeGenerator
import json

# 获取test目录路径
TEST_DIR = Path(__file__).parent
OUTPUT_DIR = TEST_DIR / "output"

# 创建输出目录
OUTPUT_DIR.mkdir(exist_ok=True)


def test_case_1_standard_board():
    """测试案例1：标准150mm x 500mm板"""
    print("\n" + "="*60)
    print("测试案例1：标准板 (150mm x 500mm)")
    print("="*60)
    
    generator = FoamBoardQRCodeGenerator(grid_size=50.0)
    
    result = generator.generate_complete_board_qrcode(
        board_width=150.0,
        board_length=500.0,
        board_id="BOARD-150x500-001",
        output_file=str(OUTPUT_DIR / "test_board_150x500.png"),
        qr_size_mm=25
    )
    
    print_result(result)
    
    # 测试解码
    test_decode(result['json_data'], generator)


def test_case_2_small_board():
    """测试案例2：小尺寸板 (100mm x 500mm)"""
    print("\n" + "="*60)
    print("测试案例2：小尺寸板 (100mm x 500mm)")
    print("="*60)
    
    generator = FoamBoardQRCodeGenerator(grid_size=50.0)
    
    result = generator.generate_complete_board_qrcode(
        board_width=100.0,
        board_length=500.0,
        board_id="BOARD-100x500-002",
        output_file=str(OUTPUT_DIR / "test_board_100x500.png"),
        qr_size_mm=25
    )
    
    print_result(result)
    test_decode(result['json_data'], generator)


def test_case_3_large_board():
    """测试案例3：大尺寸板 (200mm x 500mm)"""
    print("\n" + "="*60)
    print("测试案例3：大尺寸板 (200mm x 500mm)")
    print("="*60)
    
    generator = FoamBoardQRCodeGenerator(grid_size=50.0)
    
    result = generator.generate_complete_board_qrcode(
        board_width=200.0,
        board_length=500.0,
        board_id="BOARD-200x500-003",
        output_file=str(OUTPUT_DIR / "test_board_200x500.png"),
        qr_size_mm=30
    )
    
    print_result(result)
    test_decode(result['json_data'], generator)


def test_case_4_custom_data():
    """测试案例4：自定义区域数据"""
    print("\n" + "="*60)
    print("测试案例4：自定义区域数据")
    print("="*60)
    
    generator = FoamBoardQRCodeGenerator(grid_size=50.0)
    
    # 创建自定义数据：3列x10行=30个区域
    # 使用特定的编码规则
    custom_data = []
    for row in range(10):
        for col in range(3):
            # 编码格式：A区=100+序号，B区=200+序号，C区=300+序号
            if col == 0:
                code = 100 + row + 1
            elif col == 1:
                code = 200 + row + 1
            else:
                code = 300 + row + 1
            custom_data.append(code)
    
    result = generator.generate_complete_board_qrcode(
        board_width=150.0,
        board_length=500.0,
        board_id="CUSTOM-ABC-004",
        custom_data=custom_data,
        output_file=str(OUTPUT_DIR / "test_board_custom.png"),
        qr_size_mm=25
    )
    
    print_result(result)
    print(f"自定义数据示例: {custom_data[:10]}...")
    test_decode(result['json_data'], generator)


def test_case_5_production_batch():
    """测试案例5：批量生产场景"""
    print("\n" + "="*60)
    print("测试案例5：批量生产 - 生成一批板")
    print("="*60)
    
    generator = FoamBoardQRCodeGenerator(grid_size=50.0)
    
    batch_id = "20241203"
    num_boards = 5
    
    for i in range(num_boards):
        board_num = f"{i+1:04d}"
        board_id = f"{batch_id}-{board_num}"
        
        result = generator.generate_complete_board_qrcode(
            board_width=150.0,
            board_length=500.0,
            board_id=board_id,
            output_file=str(OUTPUT_DIR / f"batch_{board_id}.png"),
            qr_size_mm=25
        )
        
        print(f"\n批次 {i+1}/{num_boards}:")
        print(f"  板ID: {result['board_id']}")
        print(f"  JSON大小: {result['json_size_bytes']} 字节")
        print(f"  文件: {result['output_file']}")


def test_case_6_real_world_scenario():
    """测试案例6：真实生产场景"""
    print("\n" + "="*60)
    print("测试案例6：真实生产场景 - 模拟实际使用")
    print("="*60)
    
    generator = FoamBoardQRCodeGenerator(grid_size=50.0)
    
    # 模拟从生产系统获取的数据
    production_data = {
        "batch": "20241203-BATCH-A",
        "board_serial": "A0123",
        "width": 150.0,
        "length": 500.0,
        "quality_zones": [
            101, 102, 103,  # 第1行：优质区
            201, 202, 203,  # 第2行：优质区
            301, 302, 303,  # 第3行：优质区
            401, 402, 403,  # 第4行：良好区
            501, 502, 503,  # 第5行：良好区
            601, 602, 603,  # 第6行：良好区
            701, 702, 703,  # 第7行：合格区
            801, 802, 803,  # 第8行：合格区
            901, 902, 903,  # 第9行：合格区
            999, 999, 999,  # 第10行：待检区
        ]
    }
    
    board_id = f"{production_data['batch']}-{production_data['board_serial']}"
    
    result = generator.generate_complete_board_qrcode(
        board_width=production_data['width'],
        board_length=production_data['length'],
        board_id=board_id,
        custom_data=production_data['quality_zones'],
        output_file=str(OUTPUT_DIR / "test_production_real.png"),
        qr_size_mm=25
    )
    
    print_result(result)
    print(f"\n质量分区数据: {production_data['quality_zones']}")
    test_decode(result['json_data'], generator)
    
    # 演示如何查询特定位置
    print("\n位置查询演示：")
    board_data = generator.decode_board_qrcode(result['json_data'])
    print(f"  位置[0,0] (第1列,第1行) 的编码: {generator.get_region_code_by_position(board_data, 0, 0)}")
    print(f"  位置[2,9] (第3列,第10行) 的编码: {generator.get_region_code_by_position(board_data, 2, 9)}")
    
    # 显示网格
    grid = generator.get_region_grid(board_data)
    print("\n完整区域网格:")
    print(grid)


def print_result(result: dict):
    """打印结果信息"""
    print(f"\n生成结果：")
    print(f"  板ID: {result['board_id']}")
    print(f"  板尺寸: {result['width_mm']}mm × {result['length_mm']}mm")
    print(f"  网格: {result['grid_cols']}列 × {result['grid_rows']}行")
    print(f"  总区域数: {result['total_regions']}")
    print(f"  JSON大小: {result['json_size_bytes']} 字节")
    print(f"  输出文件: {result['output_file']}")
    print(f"\nJSON数据预览:")
    # 美化显示JSON
    json_obj = json.loads(result['json_data'])
    json_str = json.dumps(json_obj, indent=2, ensure_ascii=False)
    # 如果数据太长，截断显示
    if len(json_str) > 300:
        lines = json_str.split('\n')
        preview = '\n'.join(lines[:10]) + "\n    ... (更多数据) ...\n  ]"
        print(preview)
    else:
        print(json_str)


def test_decode(json_data: str, generator: FoamBoardQRCodeGenerator):
    """测试解码功能"""
    print("\n解码验证:")
    try:
        board_data = generator.decode_board_qrcode(json_data)
        print(f"  ✓ 解码成功")
        print(f"  板ID: {board_data['id']}")
        print(f"  网格: {board_data['w']}列 × {board_data['h']}行")
        print(f"  数据长度: {len(board_data['data'])}")
    except Exception as e:
        print(f"  ✗ 解码失败: {e}")


def main():
    """运行所有测试案例"""
    print("\n" + "="*60)
    print("发泡板二维码生成测试")
    print("="*60)
    print("本测试将生成多个示例二维码图片")
    print(f"图片保存位置: {OUTPUT_DIR}")
    
    # 运行所有测试案例
    test_case_1_standard_board()
    test_case_2_small_board()
    test_case_3_large_board()
    test_case_4_custom_data()
    test_case_5_production_batch()
    test_case_6_real_world_scenario()
    
    print("\n" + "="*60)
    print("所有测试完成！")
    print("="*60)
    print("\n生成的二维码文件列表:")
    
    # 列出生成的文件
    qr_files = sorted(OUTPUT_DIR.glob("*.png"))
    for f in qr_files:
        size = f.stat().st_size
        print(f"  {f} ({size} 字节)")
    
    print("\n提示：")
    print("  1. 可以使用手机扫描这些二维码查看内容")
    print("  2. 或使用命令行工具如 zbarimg 解码：")
    print(f"     zbarimg {OUTPUT_DIR}/test_board_150x500.png")
    print("  3. 在线二维码解码器也可以使用")


if __name__ == '__main__':
    main()

