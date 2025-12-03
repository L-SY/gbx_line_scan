#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2节点：发泡板二维码生成服务
接收板参数，生成并发布二维码
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
from pathlib import Path

from .board_qrcode_generator import FoamBoardQRCodeGenerator


class QRCodeGeneratorNode(Node):
    """发泡板二维码生成ROS2节点"""
    
    def __init__(self):
        super().__init__('qrcode_generator_node')
        
        # 声明参数
        self.declare_parameter('grid_size_mm', 50.0)
        self.declare_parameter('qr_size_mm', 25)
        self.declare_parameter('error_correction', 'M')
        self.declare_parameter('output_directory', '/tmp/foam_board_qrcodes')
        self.declare_parameter('default_board_width', 150.0)
        self.declare_parameter('default_board_length', 500.0)
        
        # 获取参数
        grid_size = self.get_parameter('grid_size_mm').value
        self.qr_size_mm = self.get_parameter('qr_size_mm').value
        self.error_correction = self.get_parameter('error_correction').value
        self.output_dir = Path(self.get_parameter('output_directory').value)
        self.default_width = self.get_parameter('default_board_width').value
        self.default_length = self.get_parameter('default_board_length').value
        
        # 创建输出目录
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 初始化生成器
        self.generator = FoamBoardQRCodeGenerator(grid_size=grid_size)
        
        # 初始化CV Bridge
        try:
            self.bridge = CvBridge()
            self.has_cv_bridge = True
        except:
            self.get_logger().warn('cv_bridge未安装，图像发布功能将不可用')
            self.has_cv_bridge = False
        
        # 创建订阅者 - 接收板参数JSON
        self.subscription = self.create_subscription(
            String,
            'board_parameters',
            self.board_parameters_callback,
            10
        )
        
        # 创建发布者
        self.qrcode_info_pub = self.create_publisher(String, 'qrcode_info', 10)
        if self.has_cv_bridge:
            self.qrcode_image_pub = self.create_publisher(Image, 'qrcode_image', 10)
        
        self.get_logger().info('发泡板二维码生成节点已启动')
        self.get_logger().info(f'输出目录: {self.output_dir}')
        self.get_logger().info(f'网格尺寸: {grid_size}mm')
        self.get_logger().info(f'二维码尺寸: {self.qr_size_mm}mm')
    
    def board_parameters_callback(self, msg: String):
        """
        处理板参数消息
        
        消息格式（JSON）:
        {
            "board_id": "可选，不提供则自动生成",
            "width": 板宽度（mm），
            "length": 板长度（mm），
            "custom_data": [可选，自定义区域数据数组]
        }
        """
        try:
            params = json.loads(msg.data)
            
            # 提取参数
            board_id = params.get('board_id', None)
            width = params.get('width', self.default_width)
            length = params.get('length', self.default_length)
            custom_data = params.get('custom_data', None)
            
            self.get_logger().info(f'收到板参数: 宽={width}mm, 长={length}mm')
            
            # 生成二维码
            output_file = self.output_dir / f'board_{board_id if board_id else "auto"}.png'
            result = self.generator.generate_complete_board_qrcode(
                board_width=width,
                board_length=length,
                board_id=board_id,
                custom_data=custom_data,
                output_file=str(output_file),
                qr_size_mm=self.qr_size_mm
            )
            
            # 发布结果信息
            info_msg = String()
            info_msg.data = json.dumps(result, ensure_ascii=False)
            self.qrcode_info_pub.publish(info_msg)
            
            self.get_logger().info(f'二维码生成成功:')
            self.get_logger().info(f'  板ID: {result["board_id"]}')
            self.get_logger().info(f'  网格: {result["grid_cols"]}列 x {result["grid_rows"]}行')
            self.get_logger().info(f'  总区域数: {result["total_regions"]}')
            self.get_logger().info(f'  JSON大小: {result["json_size_bytes"]} 字节')
            
            # 发布图像（如果支持）
            if self.has_cv_bridge:
                self.publish_qrcode_image(str(output_file))
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON解析错误: {e}')
        except Exception as e:
            self.get_logger().error(f'生成二维码时出错: {e}')
    
    def publish_qrcode_image(self, image_path: str):
        """发布二维码图像"""
        try:
            # 读取图像
            img = cv2.imread(image_path)
            if img is None:
                self.get_logger().error(f'无法读取图像: {image_path}')
                return
            
            # 转换为ROS消息
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.qrcode_image_pub.publish(img_msg)
            
            self.get_logger().info('二维码图像已发布')
        except Exception as e:
            self.get_logger().error(f'发布图像时出错: {e}')
    
    def generate_default_board(self):
        """生成默认板的二维码（用于测试）"""
        self.get_logger().info('生成默认测试板...')
        
        result = self.generator.generate_complete_board_qrcode(
            board_width=self.default_width,
            board_length=self.default_length,
            output_file=str(self.output_dir / 'default_board.png'),
            qr_size_mm=self.qr_size_mm
        )
        
        # 发布结果
        info_msg = String()
        info_msg.data = json.dumps(result, ensure_ascii=False)
        self.qrcode_info_pub.publish(info_msg)
        
        self.get_logger().info('默认板二维码已生成')


def main(args=None):
    rclpy.init(args=args)
    node = QRCodeGeneratorNode()
    
    # 生成一个默认测试板
    node.generate_default_board()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

