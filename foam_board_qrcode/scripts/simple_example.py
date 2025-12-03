#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单使用示例 - 快速生成一个发泡板二维码
"""

import sys
from pathlib import Path

# 添加包路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from foam_board_qrcode.board_qrcode_generator import FoamBoardQRCodeGenerator


def main():
    print("="*60)
    print("发泡板二维码生成 - 简单示例")
    print("="*60)
    
    # 创建生成器
    generator = FoamBoardQRCodeGenerator(grid_size=50.0)
    
    # 生成一个标准的150mm x 500mm板的二维码
    result = generator.generate_complete_board_qrcode(
        board_width=150.0,      # 板宽150mm
        board_length=500.0,     # 板长500mm
        board_id="EXAMPLE-001", # 板ID
        output_file="/tmp/simple_example_qrcode.png",
        qr_size_mm=25
    )
    
    # 显示结果
    print(f"\n✓ 二维码生成成功！")
    print(f"\n板信息：")
    print(f"  板ID: {result['board_id']}")
    print(f"  板尺寸: {result['width_mm']}mm × {result['length_mm']}mm")
    print(f"  网格: {result['grid_cols']}列 × {result['grid_rows']}行")
    print(f"  总区域数: {result['total_regions']}")
    print(f"\n二维码信息：")
    print(f"  JSON数据大小: {result['json_size_bytes']} 字节")
    print(f"  输出文件: {result['output_file']}")
    print(f"\nJSON内容：")
    print(f"  {result['json_data']}")
    
    # 演示解码
    print(f"\n" + "="*60)
    print("解码演示")
    print("="*60)
    
    board_data = generator.decode_board_qrcode(result['json_data'])
    print(f"✓ 解码成功")
    print(f"\n区域查询示例：")
    
    # 查询几个位置
    positions = [(0, 0), (1, 0), (2, 9)]
    for col, row in positions:
        code = generator.get_region_code_by_position(board_data, col, row)
        print(f"  位置[列{col},行{row}] 的编码: {code}")
    
    # 显示完整网格
    print(f"\n完整区域编码网格：")
    grid = generator.get_region_grid(board_data)
    print(grid)
    
    print(f"\n" + "="*60)
    print("提示：可以用手机扫描生成的二维码查看内容")
    print(f"二维码文件位置: {result['output_file']}")
    print("="*60)


if __name__ == '__main__':
    main()

