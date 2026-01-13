#!/usr/bin/env python3
"""
图像裁切工具
将图像裁切成5x5cm大小的小图，并按蛇形顺序编号：
- 从左上角开始为1，向右递增
- 到达最右边后，下一行从右向左递增
- 依次交替

使用方法:
    python crop_image.py <image_path> [--cols COLS] [--output_dir OUTPUT_DIR]

参数:
    image_path: 输入图像路径
    --cols: 每行裁切数量（默认3，即每行3个5cm图像）
    --output_dir: 输出目录（默认为图像所在目录）
"""

import argparse
import csv
import os
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def generate_diagram(
    img: Image.Image,
    crop_size_px: int,
    actual_rows: int,
    actual_cols: int,
    output_path: str
):
    """
    生成裁切示意图，在原图上绘制网格线和编号
    
    Args:
        img: 原始图像
        crop_size_px: 裁切尺寸（像素）
        actual_rows: 行数
        actual_cols: 列数
        output_path: 输出路径
    """
    # 复制原图
    diagram = img.copy()
    draw = ImageDraw.Draw(diagram)
    
    # 计算字体大小（根据裁切尺寸自适应）
    font_size = max(crop_size_px // 4, 20)
    try:
        # 尝试加载系统字体
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", font_size)
    except (OSError, IOError):
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", font_size)
        except (OSError, IOError):
            # 使用默认字体
            font = ImageFont.load_default()
    
    # 线条颜色和宽度
    line_color = (255, 0, 0)  # 红色
    line_width = max(crop_size_px // 100, 3)
    
    # 绘制垂直线
    for col in range(actual_cols + 1):
        x = col * crop_size_px
        draw.line([(x, 0), (x, actual_rows * crop_size_px)], fill=line_color, width=line_width)
    
    # 绘制水平线
    for row in range(actual_rows + 1):
        y = row * crop_size_px
        draw.line([(0, y), (actual_cols * crop_size_px, y)], fill=line_color, width=line_width)
    
    # 绘制编号（按蛇形顺序）
    piece_number = 1
    for row in range(actual_rows):
        if row % 2 == 0:
            col_range = range(actual_cols)
        else:
            col_range = range(actual_cols - 1, -1, -1)
        
        for col in col_range:
            # 计算文字位置（居中）
            left = col * crop_size_px
            upper = row * crop_size_px
            center_x = left + crop_size_px // 2
            center_y = upper + crop_size_px // 2
            
            text = str(piece_number)
            
            # 获取文字边界框
            bbox = draw.textbbox((0, 0), text, font=font)
            text_width = bbox[2] - bbox[0]
            text_height = bbox[3] - bbox[1]
            
            text_x = center_x - text_width // 2
            text_y = center_y - text_height // 2
            
            # 绘制文字（绿色）
            draw.text((text_x, text_y), text, fill=(0, 255, 0), font=font)
            
            piece_number += 1
    
    # 保存示意图
    diagram.save(output_path, "JPEG", quality=95)
    print(f"示意图已保存: {output_path}")


def crop_image_grid(
    image_path: str,
    cols: int = 3,
    output_dir: str = None
):
    """
    将图像按网格裁切，并按蛇形顺序编号
    
    Args:
        image_path: 输入图像路径
        cols: 每行的裁切数量（每个代表5cm）
        output_dir: 输出目录
    """
    # 加载图像
    img = Image.open(image_path)
    img_width, img_height = img.size
    
    # 根据每行的5cm图像数量计算裁切尺寸（像素）
    # 假设图像宽度正好对应 cols 个 5cm 区域
    crop_size_px = img_width // cols
    
    print(f"图像尺寸: {img_width} x {img_height} 像素")
    print(f"每行 {cols} 个5cm图像，裁切尺寸: {crop_size_px} x {crop_size_px} 像素")
    
    # 计算可裁切的行数
    actual_cols = cols
    actual_rows = img_height // crop_size_px
    
    print(f"裁切网格: {actual_rows} 行 x {actual_cols} 列")
    print(f"总裁切数: {actual_rows * actual_cols}")
    
    # 设置输出目录
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(image_path))
    os.makedirs(output_dir, exist_ok=True)
    
    # 获取原始图像名称（不含扩展名）
    base_name = Path(image_path).stem
    
    # 裁切图像并保存
    cropped_images = []
    piece_number = 1
    
    for row in range(actual_rows):
        # 确定该行的列顺序（蛇形：偶数行从左到右，奇数行从右到左）
        if row % 2 == 0:
            # 偶数行（0, 2, 4...）：从左到右
            col_range = range(actual_cols)
        else:
            # 奇数行（1, 3, 5...）：从右到左
            col_range = range(actual_cols - 1, -1, -1)
        
        for col in col_range:
            # 计算裁切区域
            left = col * crop_size_px
            upper = row * crop_size_px
            right = left + crop_size_px
            lower = upper + crop_size_px
            
            # 裁切
            cropped = img.crop((left, upper, right, lower))
            
            # 生成文件名
            cropped_filename = f"cropped_{base_name}_{piece_number}.jpg"
            cropped_path = os.path.join(output_dir, cropped_filename)
            
            # 保存裁切图像
            cropped.save(cropped_path, "JPEG", quality=95)
            
            cropped_images.append({
                "image_name": cropped_filename,
                "piece_number": piece_number,
                "row": row + 1,
                "col": col + 1,
                "position": f"({left}, {upper}) - ({right}, {lower})"
            })
            
            print(f"  裁切 #{piece_number}: 行{row + 1} 列{col + 1} -> {cropped_filename}")
            piece_number += 1
    
    # 生成CSV文件
    csv_filename = f"labels_{base_name}.csv"
    csv_path = os.path.join(output_dir, csv_filename)
    
    with open(csv_path, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['image_name', 'label'])
        for item in cropped_images:
            writer.writerow([item['image_name'], ''])  # label为空，等待填写
    
    # 生成裁切示意图
    diagram_filename = f"diagram_{base_name}.jpg"
    diagram_path = os.path.join(output_dir, diagram_filename)
    generate_diagram(img, crop_size_px, actual_rows, actual_cols, diagram_path)
    
    print(f"\n裁切完成!")
    print(f"输出目录: {output_dir}")
    print(f"CSV文件: {csv_path}")
    print(f"示意图: {diagram_path}")
    print(f"总共裁切: {len(cropped_images)} 张图片")
    
    return cropped_images, csv_path


def main():
    parser = argparse.ArgumentParser(
        description='将图像裁切成5x5cm大小的小图，按蛇形顺序编号'
    )
    parser.add_argument(
        'image_path',
        type=str,
        help='输入图像路径'
    )
    parser.add_argument(
        '--cols',
        type=int,
        default=3,
        help='每行5cm图像数量（默认3）'
    )
    parser.add_argument(
        '--output_dir',
        type=str,
        default=None,
        help='输出目录（默认为图像所在目录）'
    )
    
    args = parser.parse_args()
    
    if not os.path.exists(args.image_path):
        print(f"错误: 图像文件不存在: {args.image_path}")
        return
    
    crop_image_grid(
        image_path=args.image_path,
        cols=args.cols,
        output_dir=args.output_dir
    )


if __name__ == '__main__':
    main()
