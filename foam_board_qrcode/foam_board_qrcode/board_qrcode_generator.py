#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
发泡板二维码生成核心模块
功能：将整板区域信息编码为JSON并生成二维码
"""

import json
import qrcode
from datetime import datetime
from typing import List, Tuple, Optional
import numpy as np
from PIL import Image


class FoamBoardQRCodeGenerator:
    """发泡板二维码生成器"""
    
    def __init__(self, grid_size: float = 50.0):
        """
        初始化生成器
        
        Args:
            grid_size: 网格尺寸（mm），默认50x50mm
        """
        self.grid_size = grid_size
        
    def calculate_grid_dimensions(self, board_width: float, board_length: float) -> Tuple[int, int]:
        """
        计算板的网格数量
        
        Args:
            board_width: 板宽度（mm），范围100-200mm
            board_length: 板长度（mm），通常500mm
            
        Returns:
            (列数, 行数)
        """
        cols = int(board_width / self.grid_size)
        rows = int(board_length / self.grid_size)
        return cols, rows
    
    def generate_board_id(self, prefix: str = "") -> str:
        """
        生成唯一板ID
        
        Args:
            prefix: ID前缀
            
        Returns:
            格式如：20240119-0012 或自定义前缀
        """
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        if prefix:
            return f"{prefix}-{timestamp}"
        return timestamp
    
    def create_region_data(self, cols: int, rows: int, 
                          start_code: int = 101, 
                          custom_data: Optional[List[int]] = None) -> List[int]:
        """
        创建区域数据数组（行优先顺序）
        
        Args:
            cols: 列数
            rows: 行数
            start_code: 起始编码（默认101）
            custom_data: 自定义区域数据，如果提供则使用此数据
            
        Returns:
            区域编码列表
        """
        total_regions = cols * rows
        
        if custom_data is not None:
            if len(custom_data) != total_regions:
                raise ValueError(f"自定义数据长度({len(custom_data)})必须等于区域总数({total_regions})")
            return custom_data
        
        # 默认按行优先生成连续编码
        data = []
        for row in range(rows):
            for col in range(cols):
                # 编码格式：行号*100 + 列号
                code = start_code + row * 100 + col
                data.append(code)
        
        return data
    
    def create_board_data_json(self, board_id: str, cols: int, rows: int, 
                              region_data: List[int]) -> str:
        """
        创建板数据JSON字符串
        
        Args:
            board_id: 板ID
            cols: 列数
            rows: 行数
            region_data: 区域数据数组
            
        Returns:
            精简的JSON字符串
        """
        data_dict = {
            "id": board_id,
            "w": cols,
            "h": rows,
            "data": region_data
        }
        # 使用最紧凑的JSON格式
        json_str = json.dumps(data_dict, separators=(',', ':'), ensure_ascii=False)
        return json_str
    
    def generate_qrcode_image(self, json_data: str, 
                             qr_size_mm: int = 25,
                             error_correction: str = 'M') -> Image.Image:
        """
        生成二维码图像
        
        Args:
            json_data: JSON数据字符串
            qr_size_mm: 二维码目标尺寸（mm），默认25mm
            error_correction: 错误纠正级别 ('L', 'M', 'Q', 'H')
            
        Returns:
            PIL Image对象
        """
        # 错误纠正级别映射
        error_correction_map = {
            'L': qrcode.constants.ERROR_CORRECT_L,  # 7%
            'M': qrcode.constants.ERROR_CORRECT_M,  # 15%
            'Q': qrcode.constants.ERROR_CORRECT_Q,  # 25%
            'H': qrcode.constants.ERROR_CORRECT_H   # 30%
        }
        
        qr = qrcode.QRCode(
            version=None,  # 自动选择版本
            error_correction=error_correction_map.get(error_correction, 
                                                     qrcode.constants.ERROR_CORRECT_M),
            box_size=10,  # 每个小方块的像素数
            border=4,     # 边框宽度（小方块数）
        )
        
        qr.add_data(json_data)
        qr.make(fit=True)
        
        # 生成图像
        img = qr.make_image(fill_color="black", back_color="white")
        
        return img
    
    def save_qrcode(self, img: Image.Image, filename: str):
        """
        保存二维码图像
        
        Args:
            img: PIL Image对象
            filename: 保存的文件名
        """
        img.save(filename)
        print(f"二维码已保存至: {filename}")
    
    def generate_complete_board_qrcode(self, 
                                      board_width: float,
                                      board_length: float,
                                      board_id: Optional[str] = None,
                                      custom_data: Optional[List[int]] = None,
                                      output_file: str = "board_qrcode.png",
                                      qr_size_mm: int = 25) -> dict:
        """
        一站式生成完整的发泡板二维码
        
        Args:
            board_width: 板宽度（mm）
            board_length: 板长度（mm）
            board_id: 板ID，如不提供则自动生成
            custom_data: 自定义区域数据
            output_file: 输出文件名
            qr_size_mm: 二维码尺寸（mm）
            
        Returns:
            包含板信息和JSON数据的字典
        """
        # 计算网格
        cols, rows = self.calculate_grid_dimensions(board_width, board_length)
        
        # 生成板ID
        if board_id is None:
            board_id = self.generate_board_id()
        
        # 生成区域数据
        region_data = self.create_region_data(cols, rows, custom_data=custom_data)
        
        # 创建JSON
        json_data = self.create_board_data_json(board_id, cols, rows, region_data)
        
        # 生成二维码
        img = self.generate_qrcode_image(json_data, qr_size_mm)
        
        # 保存
        self.save_qrcode(img, output_file)
        
        # 返回信息
        result = {
            "board_id": board_id,
            "width_mm": board_width,
            "length_mm": board_length,
            "grid_cols": cols,
            "grid_rows": rows,
            "total_regions": cols * rows,
            "json_data": json_data,
            "json_size_bytes": len(json_data.encode('utf-8')),
            "output_file": output_file
        }
        
        return result
    
    @staticmethod
    def decode_board_qrcode(json_data: str) -> dict:
        """
        解码板二维码数据
        
        Args:
            json_data: 从二维码中读取的JSON字符串
            
        Returns:
            解析后的板数据字典
        """
        data = json.loads(json_data)
        
        # 验证数据完整性
        required_keys = ['id', 'w', 'h', 'data']
        for key in required_keys:
            if key not in data:
                raise ValueError(f"缺少必需字段: {key}")
        
        # 验证数据一致性
        expected_length = data['w'] * data['h']
        if len(data['data']) != expected_length:
            raise ValueError(f"数据长度不匹配: 预期{expected_length}, 实际{len(data['data'])}")
        
        return data
    
    @staticmethod
    def get_region_code_by_position(board_data: dict, col: int, row: int) -> int:
        """
        根据位置获取区域编码
        
        Args:
            board_data: 解码后的板数据
            col: 列索引（0开始）
            row: 行索引（0开始）
            
        Returns:
            该位置的区域编码
        """
        if col < 0 or col >= board_data['w']:
            raise ValueError(f"列索引超出范围: {col}")
        if row < 0 or row >= board_data['h']:
            raise ValueError(f"行索引超出范围: {row}")
        
        # 行优先索引
        index = row * board_data['w'] + col
        return board_data['data'][index]
    
    @staticmethod
    def get_region_grid(board_data: dict) -> np.ndarray:
        """
        将区域数据转换为2D网格数组
        
        Args:
            board_data: 解码后的板数据
            
        Returns:
            形状为(rows, cols)的numpy数组
        """
        data_array = np.array(board_data['data'])
        grid = data_array.reshape(board_data['h'], board_data['w'])
        return grid

