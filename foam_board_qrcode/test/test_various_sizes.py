#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
演示脚本：展示系统对不同尺寸板子的适应性
"""

import sys
from pathlib import Path

# 添加包路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from foam_board_qrcode.board_qrcode_generator import FoamBoardQRCodeGenerator

# 输出目录
OUTPUT_DIR = Path(__file__).parent / "output"
OUTPUT_DIR.mkdir(exist_ok=True)


def test_various_sizes():
    """测试各种不同的板子尺寸"""
    
    print("="*70)
    print("发泡板二维码 - 不同尺寸适应性演示")
    print("="*70)
    print("\n系统支持任意尺寸的板子，会自动计算网格数量！\n")
    
    generator = FoamBoardQRCodeGenerator(grid_size=50.0)
    
    # 定义各种不同尺寸的板子
    test_cases = [
        # (宽度, 长度, 描述)
        (100, 400, "窄短板"),
        (100, 500, "窄标准板"),
        (100, 600, "窄长板"),
        (150, 400, "标准窄板"),
        (150, 500, "标准板（原始规格）"),
        (150, 600, "标准长板"),
        (200, 400, "宽短板"),
        (200, 500, "宽标准板"),
        (200, 600, "宽长板"),
        (250, 500, "超宽标准板"),
        (120, 450, "定制尺寸1"),
        (180, 550, "定制尺寸2"),
    ]
    
    results = []
    
    for width, length, description in test_cases:
        print(f"\n{'-'*70}")
        print(f"测试板子: {description}")
        print(f"尺寸: {width}mm × {length}mm")
        
        # 计算网格
        cols, rows = generator.calculate_grid_dimensions(width, length)
        total_regions = cols * rows
        
        print(f"网格: {cols}列 × {rows}行 = {total_regions}个区域")
        
        # 生成二维码
        board_id = f"BOARD-{width}x{length}"
        output_file = OUTPUT_DIR / f"size_{width}x{length}.png"
        
        result = generator.generate_complete_board_qrcode(
            board_width=float(width),
            board_length=float(length),
            board_id=board_id,
            output_file=str(output_file),
            qr_size_mm=25
        )
        
        print(f"JSON大小: {result['json_size_bytes']} 字节")
        print(f"✓ 二维码已生成: {output_file.name}")
        
        results.append({
            'width': width,
            'length': length,
            'description': description,
            'cols': cols,
            'rows': rows,
            'total_regions': total_regions,
            'json_bytes': result['json_size_bytes']
        })
    
    # 打印汇总表格
    print("\n" + "="*70)
    print("所有尺寸测试完成 - 汇总表")
    print("="*70)
    print(f"{'描述':<15} {'尺寸(mm)':<12} {'网格':<10} {'区域数':<8} {'JSON(字节)':<12}")
    print("-"*70)
    
    for r in results:
        size_str = f"{r['width']}×{r['length']}"
        grid_str = f"{r['cols']}×{r['rows']}"
        print(f"{r['description']:<15} {size_str:<12} {grid_str:<10} "
              f"{r['total_regions']:<8} {r['json_bytes']:<12}")
    
    print("\n" + "="*70)
    print("✓ 结论：系统可以处理任意尺寸的板子！")
    print("  - 网格数量自动计算")
    print("  - JSON大小自动适应")
    print("  - 二维码自动生成")
    print("="*70)


def test_various_grid_sizes():
    """测试不同的网格尺寸"""
    
    print("\n\n" + "="*70)
    print("网格尺寸调整演示")
    print("="*70)
    print("\n同一块板子（150×500mm），使用不同的网格尺寸：\n")
    
    board_width = 150.0
    board_length = 500.0
    
    grid_sizes = [25.0, 50.0, 100.0]
    
    for grid_size in grid_sizes:
        print(f"\n{'-'*70}")
        print(f"网格尺寸: {grid_size}mm × {grid_size}mm")
        
        generator = FoamBoardQRCodeGenerator(grid_size=grid_size)
        
        cols, rows = generator.calculate_grid_dimensions(board_width, board_length)
        total_regions = cols * rows
        
        print(f"板尺寸: {board_width}mm × {board_length}mm")
        print(f"网格: {cols}列 × {rows}行 = {total_regions}个区域")
        
        # 生成二维码
        board_id = f"GRID{int(grid_size)}-150x500"
        output_file = OUTPUT_DIR / f"grid_{int(grid_size)}mm.png"
        
        result = generator.generate_complete_board_qrcode(
            board_width=board_width,
            board_length=board_length,
            board_id=board_id,
            output_file=str(output_file),
            qr_size_mm=25
        )
        
        print(f"JSON大小: {result['json_size_bytes']} 字节")
        print(f"✓ 二维码已生成: {output_file.name}")
    
    print("\n" + "="*70)
    print("✓ 结论：可以根据需求调整网格精度！")
    print("  - 小网格 → 更多区域，更精细的标识")
    print("  - 大网格 → 更少区域，JSON更小")
    print("="*70)


def main():
    """运行所有演示"""
    test_various_sizes()
    test_various_grid_sizes()
    
    print("\n\n" + "="*70)
    print("💡 使用建议")
    print("="*70)
    print("""
1. 板子尺寸变化时：
   - 直接修改 board_width 和 board_length 参数即可
   - 系统会自动计算网格数量
   - 无需修改任何代码

2. 如果区域太多或太少：
   - 调整 grid_size 参数
   - 例如：grid_size=25.0 会得到4倍的区域数量

3. JSON大小控制：
   - 一般情况下，30-50个区域（100-200字节）最合适
   - 超过100个区域时，考虑增大网格尺寸

4. 实际使用：
   - 可以为不同产品线使用不同的尺寸配置
   - 在ROS2参数文件中设置默认值
   - 运行时动态传入具体尺寸
    """)
    print("="*70)
    
    # 显示所有生成的文件
    print("\n生成的文件：")
    for f in sorted(OUTPUT_DIR.glob("*.png")):
        if f.name.startswith(('size_', 'grid_')):
            print(f"  ✓ {f.name}")


if __name__ == '__main__':
    main()

