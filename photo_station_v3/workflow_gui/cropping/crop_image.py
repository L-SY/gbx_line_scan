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
import tkinter as tk
from tkinter import filedialog, messagebox

from PIL import Image, ImageDraw, ImageFont, ImageTk


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


def crop_height(image: Image.Image, top_crop: int = 0, bottom_crop: int = 0) -> Image.Image:
    """
    在高度上下裁剪指定像素
    
    Args:
        image: 输入图像
        top_crop: 上边裁剪的像素数
        bottom_crop: 下边裁剪的像素数
    
    Returns:
        裁剪后的图像
    """
    if top_crop <= 0 and bottom_crop <= 0:
        return image
    
    width, height = image.size
    
    top = top_crop if top_crop > 0 else 0
    bottom = height - bottom_crop if bottom_crop > 0 else height
    
    if top >= bottom:
        return image
    
    return image.crop((0, top, width, bottom))


def crop_image_grid(
    image_path: str,
    cols: int = 3,
    output_dir: str = None,
    height_crop_pixels: int = 0,
    top_crop: int = None,
    bottom_crop: int = None
):
    """
    将图像按网格裁切，并按蛇形顺序编号
    
    Args:
        image_path: 输入图像路径
        cols: 每行的裁切数量（每个代表5cm）
        output_dir: 输出目录
        height_crop_pixels: 高度裁剪像素数（上下各裁剪，兼容旧参数）
        top_crop: 上边裁剪像素数（优先使用）
        bottom_crop: 下边裁剪像素数（优先使用）
    """
    # 加载图像
    img = Image.open(image_path)
    
    # 先进行高度裁剪
    if top_crop is not None and bottom_crop is not None:
        # 使用新的分别设置方式
        if top_crop > 0 or bottom_crop > 0:
            img = crop_height(img, top_crop, bottom_crop)
            print(f"高度裁剪: 上边裁剪 {top_crop} 像素, 下边裁剪 {bottom_crop} 像素")
    elif height_crop_pixels > 0:
        # 兼容旧参数（上下各裁剪相同像素）
        img = crop_height(img, height_crop_pixels, height_crop_pixels)
        print(f"高度裁剪: 上下各裁剪 {height_crop_pixels} 像素")
    
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


class HeightCropGUI:
    """高度裁剪交互界面"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("图像高度裁剪工具")
        self.root.geometry("1400x900")
        self.root.configure(bg='#ffffff')
        
        self.original_image = None
        self.display_image = None
        self.top_crop = tk.IntVar(value=0)
        self.bottom_crop = tk.IntVar(value=0)
        self.image_path = None
        self.confirmed = False
        
        self.setup_ui()
        
    def setup_ui(self):
        # 左侧控制面板 - 白色背景
        control_frame = tk.Frame(self.root, width=320, bg='#ffffff')
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=15, pady=15)
        control_frame.pack_propagate(False)
        
        # 标题
        title_label = tk.Label(control_frame, text="图像裁剪工具", 
                              font=('Arial', 18, 'bold'), bg='#ffffff', 
                              fg='#000000', pady=20)
        title_label.pack()
        
        # 分隔线
        separator1 = tk.Frame(control_frame, height=1, bg='#cccccc')
        separator1.pack(fill=tk.X, padx=10, pady=15)
        
        # 加载图片按钮 - 黑白灰样式
        load_btn = tk.Button(control_frame, text="加载图片", command=self.load_image, 
                            width=24, height=2, font=('Arial', 11),
                            bg='#000000', fg='#ffffff', activebackground='#333333',
                            activeforeground='#ffffff', relief=tk.FLAT, cursor='hand2',
                            bd=0, padx=5, pady=5)
        load_btn.pack(pady=15)
        
        # 裁剪设置区域
        crop_section = tk.LabelFrame(control_frame, text="裁剪设置", 
                                     font=('Arial', 12, 'bold'),
                                     bg='#ffffff', fg='#000000', 
                                     padx=15, pady=15, relief=tk.FLAT, bd=1,
                                     highlightbackground='#cccccc', highlightthickness=1)
        crop_section.pack(pady=15, padx=10, fill=tk.X)
        
        # 上裁剪设置
        top_frame = tk.Frame(crop_section, bg='#ffffff')
        top_frame.pack(fill=tk.X, pady=12)
        
        tk.Label(top_frame, text="上边裁剪", font=('Arial', 10), 
                bg='#ffffff', fg='#000000').pack(anchor=tk.W, pady=(0, 5))
        
        top_input_frame = tk.Frame(top_frame, bg='#ffffff')
        top_input_frame.pack(fill=tk.X, pady=5)
        
        self.top_entry = tk.Entry(top_input_frame, textvariable=self.top_crop, 
                                  width=10, font=('Arial', 11),
                                  bg='#ffffff', fg='#000000', 
                                  insertbackground='#000000', 
                                  relief=tk.SOLID, bd=1, highlightthickness=1,
                                  highlightbackground='#cccccc', highlightcolor='#000000')
        self.top_entry.pack(side=tk.LEFT, padx=(0, 5))
        self.top_entry.bind('<KeyRelease>', self.on_crop_change)
        
        tk.Label(top_input_frame, text="像素", font=('Arial', 9), 
                bg='#ffffff', fg='#666666').pack(side=tk.LEFT)
        
        self.top_scale = tk.Scale(top_frame, from_=0, to=500, 
                                  orient=tk.HORIZONTAL, variable=self.top_crop,
                                  command=self.on_crop_change, length=240,
                                  bg='#ffffff', fg='#000000', 
                                  troughcolor='#e0e0e0', 
                                  activebackground='#666666',
                                  highlightthickness=0, relief=tk.FLAT,
                                  sliderrelief=tk.FLAT, sliderlength=20)
        self.top_scale.pack(fill=tk.X, pady=8)
        
        # 分隔线
        separator2 = tk.Frame(crop_section, height=1, bg='#e0e0e0')
        separator2.pack(fill=tk.X, pady=12)
        
        # 下裁剪设置
        bottom_frame = tk.Frame(crop_section, bg='#ffffff')
        bottom_frame.pack(fill=tk.X, pady=12)
        
        tk.Label(bottom_frame, text="下边裁剪", font=('Arial', 10), 
                bg='#ffffff', fg='#000000').pack(anchor=tk.W, pady=(0, 5))
        
        bottom_input_frame = tk.Frame(bottom_frame, bg='#ffffff')
        bottom_input_frame.pack(fill=tk.X, pady=5)
        
        self.bottom_entry = tk.Entry(bottom_input_frame, textvariable=self.bottom_crop, 
                                     width=10, font=('Arial', 11),
                                     bg='#ffffff', fg='#000000', 
                                     insertbackground='#000000', 
                                     relief=tk.SOLID, bd=1, highlightthickness=1,
                                     highlightbackground='#cccccc', highlightcolor='#000000')
        self.bottom_entry.pack(side=tk.LEFT, padx=(0, 5))
        self.bottom_entry.bind('<KeyRelease>', self.on_crop_change)
        
        tk.Label(bottom_input_frame, text="像素", font=('Arial', 9), 
                bg='#ffffff', fg='#666666').pack(side=tk.LEFT)
        
        self.bottom_scale = tk.Scale(bottom_frame, from_=0, to=500, 
                                     orient=tk.HORIZONTAL, variable=self.bottom_crop,
                                     command=self.on_crop_change, length=240,
                                     bg='#ffffff', fg='#000000', 
                                     troughcolor='#e0e0e0', 
                                     activebackground='#666666',
                                     highlightthickness=0, relief=tk.FLAT,
                                     sliderrelief=tk.FLAT, sliderlength=20)
        self.bottom_scale.pack(fill=tk.X, pady=8)
        
        # 信息显示区域
        info_section = tk.LabelFrame(control_frame, text="图像信息", 
                                     font=('Arial', 12, 'bold'),
                                     bg='#ffffff', fg='#000000', 
                                     padx=15, pady=15, relief=tk.FLAT, bd=1,
                                     highlightbackground='#cccccc', highlightthickness=1)
        info_section.pack(pady=15, padx=10, fill=tk.BOTH, expand=True)
        
        self.info_label = tk.Label(info_section, text="请加载图片", 
                                   font=('Arial', 10), bg='#ffffff', fg='#000000',
                                   wraplength=260, justify=tk.LEFT, anchor=tk.NW)
        self.info_label.pack(fill=tk.BOTH, expand=True)
        
        # 确认按钮 - 黑白灰样式
        confirm_btn = tk.Button(control_frame, text="确认裁剪参数", 
                               command=self.confirm_crop, width=24, height=2, 
                               font=('Arial', 11, 'bold'),
                               bg='#333333', fg='#ffffff', activebackground='#000000',
                               activeforeground='#ffffff', relief=tk.FLAT, cursor='hand2',
                               bd=0, state=tk.DISABLED)
        confirm_btn.pack(pady=15)
        self.confirm_btn = confirm_btn
        
        # 右侧图片显示区域 - 浅灰背景
        image_frame = tk.Frame(self.root, bg='#f5f5f5')
        image_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=15, pady=15)
        
        # 创建画布容器 - 居中显示
        canvas_container = tk.Frame(image_frame, bg='#f5f5f5')
        canvas_container.pack(fill=tk.BOTH, expand=True)
        
        # 创建滚动条
        canvas_frame = tk.Frame(canvas_container, bg='#f5f5f5')
        canvas_frame.pack(fill=tk.BOTH, expand=True)
        
        self.canvas = tk.Canvas(canvas_frame, bg='#f5f5f5', cursor='crosshair',
                               highlightthickness=0)
        scrollbar_v = tk.Scrollbar(canvas_frame, orient=tk.VERTICAL, 
                                   command=self.canvas.yview,
                                   bg='#cccccc', troughcolor='#f5f5f5',
                                   activebackground='#999999', width=12)
        scrollbar_h = tk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL, 
                                   command=self.canvas.xview,
                                   bg='#cccccc', troughcolor='#f5f5f5',
                                   activebackground='#999999', width=12)
        
        self.canvas.configure(yscrollcommand=scrollbar_v.set, xscrollcommand=scrollbar_h.set)
        
        scrollbar_v.pack(side=tk.RIGHT, fill=tk.Y)
        scrollbar_h.pack(side=tk.BOTTOM, fill=tk.X)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 绑定窗口大小变化事件
        self.root.bind('<Configure>', self.on_window_resize)
        
    def load_image(self):
        file_path = filedialog.askopenfilename(
            title="选择图片",
            filetypes=[("图片文件", "*.jpg *.jpeg *.png *.bmp *.tiff *.tif"), ("所有文件", "*.*")]
        )
        
        if file_path:
            try:
                self.original_image = Image.open(file_path)
                self.image_path = file_path
                self.update_display()
                self.confirm_btn.config(state=tk.NORMAL)
                
                # 更新滑块最大值
                max_crop = min(self.original_image.height // 2, 1000)
                self.top_scale.config(to=max_crop)
                self.bottom_scale.config(to=max_crop)
                
                self.update_info()
            except Exception as e:
                messagebox.showerror("错误", f"无法加载图片: {str(e)}")
    
    def on_crop_change(self, event=None):
        if self.original_image is not None:
            self.update_display()
            self.update_info()
    
    def update_display(self):
        if self.original_image is None:
            return
        
        top_crop = self.top_crop.get()
        bottom_crop = self.bottom_crop.get()
        cropped_image = crop_height(self.original_image, top_crop, bottom_crop)
        
        # 获取画布实际大小
        self.canvas.update_idletasks()
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        
        if canvas_width > 1 and canvas_height > 1:
            # 计算缩放比例，保持宽高比，适应画布
            img_width, img_height = cropped_image.size
            scale_w = (canvas_width - 40) / img_width
            scale_h = (canvas_height - 40) / img_height
            scale = min(scale_w, scale_h, 1.0)  # 不放大，只缩小
            
            display_width = int(img_width * scale)
            display_height = int(img_height * scale)
            
            display_img = cropped_image.resize((display_width, display_height), Image.Resampling.LANCZOS)
        else:
            display_img = cropped_image
            display_width, display_height = display_img.size
        
        # 转换为PhotoImage
        self.display_image = ImageTk.PhotoImage(display_img)
        
        # 清除画布
        self.canvas.delete("all")
        
        # 计算居中位置
        x = (canvas_width - display_width) // 2 if canvas_width > display_width else 0
        y = (canvas_height - display_height) // 2 if canvas_height > display_height else 0
        
        # 居中显示图片
        self.canvas.create_image(x, y, anchor=tk.NW, image=self.display_image)
        
        # 更新滚动区域（包含整个图片区域）
        self.canvas.config(scrollregion=(0, 0, max(canvas_width, display_width), max(canvas_height, display_height)))
    
    def update_info(self):
        if self.original_image is None:
            return
        
        top_crop = self.top_crop.get()
        bottom_crop = self.bottom_crop.get()
        orig_width, orig_height = self.original_image.size
        
        total_crop = top_crop + bottom_crop
        
        if total_crop >= orig_height:
            info_text = f"警告: 裁剪像素过大\n\n原始尺寸:\n{orig_width} × {orig_height} 像素\n\n裁剪设置:\n上: {top_crop} 像素\n下: {bottom_crop} 像素\n总计: {total_crop} 像素\n\n剩余高度: 0"
        else:
            new_height = orig_height - total_crop
            info_text = f"原始尺寸:\n{orig_width} × {orig_height} 像素\n\n裁剪设置:\n上: {top_crop} 像素\n下: {bottom_crop} 像素\n总计: {total_crop} 像素\n\n新尺寸:\n{orig_width} × {new_height} 像素\n\n裁剪后高度: {new_height} 像素"
        
        self.info_label.config(text=info_text)
    
    def on_window_resize(self, event):
        if event.widget == self.root:
            self.root.after(100, self.update_display)
    
    def confirm_crop(self):
        """确认裁剪参数并返回"""
        top_crop = self.top_crop.get()
        bottom_crop = self.bottom_crop.get()
        
        if self.original_image is None:
            messagebox.showwarning("警告", "请先加载图片")
            return
        
        total_crop = top_crop + bottom_crop
        if total_crop >= self.original_image.height:
            messagebox.showwarning("警告", "裁剪像素过大，总裁剪量不能超过图像高度")
            return
        
        self.confirmed = True
        self.root.quit()
        self.root.destroy()


def show_crop_gui(image_path: str = None):
    """
    显示高度裁剪GUI界面
    
    Args:
        image_path: 可选，初始图片路径
    
    Returns:
        (image_path, top_crop, bottom_crop) 元组，如果用户取消则返回 (None, 0, 0)
    """
    root = tk.Tk()
    app = HeightCropGUI(root)
    
    if image_path and os.path.exists(image_path):
        app.original_image = Image.open(image_path)
        app.image_path = image_path
        app.update_display()
        app.confirm_btn.config(state=tk.NORMAL)
        max_crop = min(app.original_image.height // 2, 1000)
        app.top_scale.config(to=max_crop)
        app.bottom_scale.config(to=max_crop)
        app.update_info()
    
    root.mainloop()
    
    # 检查用户是否确认了
    if hasattr(app, 'confirmed') and app.confirmed and app.image_path:
        return (app.image_path, app.top_crop.get(), app.bottom_crop.get())
    
    return (None, 0, 0)


def main():
    parser = argparse.ArgumentParser(
        description='将图像裁切成5x5cm大小的小图，按蛇形顺序编号'
    )
    parser.add_argument(
        'image_path',
        type=str,
        nargs='?',
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
    parser.add_argument(
        '--height_crop',
        type=int,
        default=0,
        help='高度裁剪像素数（上下各裁剪，默认0）'
    )
    parser.add_argument(
        '--gui',
        action='store_true',
        help='使用GUI界面交互式设置裁剪参数'
    )
    
    args = parser.parse_args()
    
    # 如果使用GUI模式
    if args.gui:
        gui_image_path, top_crop, bottom_crop = show_crop_gui(args.image_path)
        
        if gui_image_path is None:
            # 用户取消了GUI
            return
        
        # 使用GUI中选择的图片路径和裁剪参数
        args.image_path = gui_image_path
        # 为了兼容旧代码，使用top_crop作为height_crop（如果上下相同）
        # 实际裁剪会在crop_image_grid中使用top和bottom分别处理
        args.height_crop = top_crop if top_crop == bottom_crop else 0
        args.top_crop = top_crop
        args.bottom_crop = bottom_crop
    
    if args.image_path is None:
        parser.print_help()
        return
    
    if not os.path.exists(args.image_path):
        print(f"错误: 图像文件不存在: {args.image_path}")
        return
    
    # 传递裁剪参数
    top_crop = getattr(args, 'top_crop', None)
    bottom_crop = getattr(args, 'bottom_crop', None)
    
    crop_image_grid(
        image_path=args.image_path,
        cols=args.cols,
        output_dir=args.output_dir,
        height_crop_pixels=args.height_crop,
        top_crop=top_crop,
        bottom_crop=bottom_crop
    )


if __name__ == '__main__':
    main()
