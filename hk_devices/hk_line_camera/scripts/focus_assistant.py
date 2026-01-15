#!/usr/bin/env python3
"""
对焦辅助工具 - 通过拉普拉斯方差评估图像清晰度
"""

import sys
import argparse
from collections import deque
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
import numpy as np

# cv_bridge 兼容性处理
try:
    from cv_bridge import CvBridge
    USE_CV_BRIDGE = True
except (ImportError, AttributeError):
    USE_CV_BRIDGE = False
    print("警告: cv_bridge不可用，使用numpy直接转换")


def ros_image_to_cv2(msg):
    """将ROS Image消息转换为OpenCV图像（不依赖cv_bridge）"""
    dtype = np.uint8
    if msg.encoding in ['mono16', '16UC1']:
        dtype = np.uint16
    elif msg.encoding in ['32FC1']:
        dtype = np.float32
    
    # 确定通道数
    if msg.encoding in ['mono8', 'mono16', '8UC1', '16UC1', '32FC1']:
        channels = 1
    elif msg.encoding in ['bgr8', 'rgb8', '8UC3']:
        channels = 3
    elif msg.encoding in ['bgra8', 'rgba8', '8UC4']:
        channels = 4
    else:
        channels = 1  # 默认单通道
    
    # 转换
    img = np.frombuffer(msg.data, dtype=dtype)
    if channels == 1:
        img = img.reshape((msg.height, msg.width))
    else:
        img = img.reshape((msg.height, msg.width, channels))
    
    # rgb转bgr
    if msg.encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif msg.encoding == 'rgba8':
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA)
    
    return img


def scan_image_topics(node):
    """扫描所有Image类型的话题"""
    topics = []
    try:
        topic_names_and_types = node.get_topic_names_and_types()
        for topic_name, topic_types in topic_names_and_types:
            if 'sensor_msgs/msg/Image' in topic_types:
                topics.append(topic_name)
        topics.sort()
    except Exception as e:
        node.get_logger().warn(f'扫描话题失败: {e}')
    return topics


class FocusAssistant(Node):
    def __init__(self, topic_name=None, history_size=200, gui_mode=True):
        super().__init__('focus_assistant')
        
        self.gui_mode = gui_mode
        self.gui_initialized = False
        self.history_size = history_size
        self.current_topic = topic_name
        self.selecting_topic = False
        self.available_topics = []
        
        # cv_bridge
        if USE_CV_BRIDGE:
            self.bridge = CvBridge()
        
        # 清晰度历史
        self.sharpness_history = deque(maxlen=history_size)
        self.best_sharpness = 0.0
        self.frame_count = 0
        
        # 帧率计算
        self.fps_start_time = time.time()
        self.fps_count = 0
        self.current_fps = 0.0
        
        # 终端输出计时
        self.last_print_time = time.time()
        
        # 趋势图
        self.trend_width = 800
        self.trend_height = 300
        
        # 订阅（如果提供了话题）
        self.subscription = None
        if topic_name:
            self.subscribe_to_topic(topic_name)
        else:
            # 如果没有提供话题，扫描并选择
            self.available_topics = scan_image_topics(self)
            if self.available_topics:
                self.selecting_topic = True
                self.get_logger().info(f'发现 {len(self.available_topics)} 个图像话题，等待选择')
            else:
                self.get_logger().warn('未发现图像话题')
        
        self.get_logger().info(f'GUI模式: {gui_mode}')
    
    def subscribe_to_topic(self, topic_name):
        """订阅指定话题"""
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
        
        self.subscription = self.create_subscription(
            Image, topic_name, self.image_callback, 10)
        self.current_topic = topic_name
        self.get_logger().info(f'订阅话题: {topic_name}')
        
        # 重置统计
        self.sharpness_history.clear()
        self.best_sharpness = 0.0
        self.frame_count = 0

    def init_gui(self):
        """延迟初始化GUI窗口"""
        if self.gui_initialized or not self.gui_mode:
            return True
        
        try:
            cv2.namedWindow('Focus Assistant', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Focus Assistant', 1200, 800)
            self.gui_initialized = True
            self.get_logger().info('GUI初始化成功')
            return True
        except Exception as e:
            self.get_logger().warn(f'GUI初始化失败: {e}')
            self.gui_mode = False
            return False
    
    def create_topic_selection_display(self):
        """创建话题选择界面"""
        width, height = 600, 500
        img = np.ones((height, width, 3), dtype=np.uint8) * 40
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        title = "选择图像话题 (Select Image Topic)"
        cv2.putText(img, title, (20, 40), font, 0.8, (255, 255, 255), 2)
        
        if not self.available_topics:
            cv2.putText(img, "未发现图像话题", (20, 100), font, 0.7, (0, 0, 255), 2)
            cv2.putText(img, "按 's' 重新扫描", (20, 140), font, 0.6, (200, 200, 200), 1)
        else:
            y = 80
            for i, topic in enumerate(self.available_topics):
                if i >= 9:  # 只显示前9个（数字键1-9）
                    break
                
                # 高亮当前话题
                if topic == self.current_topic:
                    color = (0, 255, 0)
                    prefix = "> "
                else:
                    color = (200, 200, 200)
                    prefix = "  "
                
                text = f"{i+1}. {prefix}{topic}"
                cv2.putText(img, text, (20, y), font, 0.6, color, 1)
                y += 35
            
            if len(self.available_topics) > 9:
                cv2.putText(img, f"... 还有 {len(self.available_topics)-9} 个话题", 
                           (20, y), font, 0.5, (150, 150, 150), 1)
        
        # 说明文字
        y = height - 100
        cv2.putText(img, "操作说明:", (20, y), font, 0.6, (255, 255, 0), 1)
        y += 25
        cv2.putText(img, "数字键 1-9: 选择话题", (20, y), font, 0.5, (200, 200, 200), 1)
        y += 20
        cv2.putText(img, "s: 重新扫描话题", (20, y), font, 0.5, (200, 200, 200), 1)
        y += 20
        cv2.putText(img, "q: 退出", (20, y), font, 0.5, (200, 200, 200), 1)
        
        return img

    def calculate_sharpness(self, image):
        """计算图像清晰度（拉普拉斯方差）"""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        sharpness = laplacian.var()
        
        return sharpness, laplacian

    def create_display(self, image, laplacian, sharpness):
        """创建显示画面：原图+拉普拉斯+趋势图"""
        display_h = 400
        
        # 1. 处理原图
        if len(image.shape) == 2:
            orig = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            orig = image.copy()
        
        # 缩放原图
        scale = display_h / orig.shape[0]
        orig_w = int(orig.shape[1] * scale)
        orig_resized = cv2.resize(orig, (orig_w, display_h))
        
        # 2. 处理拉普拉斯图
        lap_abs = np.abs(laplacian)
        if lap_abs.max() > 0:
            lap_norm = (lap_abs / lap_abs.max() * 255).astype(np.uint8)
        else:
            lap_norm = np.zeros_like(lap_abs, dtype=np.uint8)
        lap_colored = cv2.applyColorMap(lap_norm, cv2.COLORMAP_JET)
        lap_resized = cv2.resize(lap_colored, (orig_w, display_h))
        
        # 3. 趋势图
        trend = self.create_trend_graph(sharpness)
        trend_resized = cv2.resize(trend, (orig_w, display_h))
        
        # 垂直拼接
        combined = np.vstack([orig_resized, lap_resized, trend_resized])
        
        # 添加文字信息
        font = cv2.FONT_HERSHEY_SIMPLEX
        info_lines = [
            f'Topic: {self.current_topic}',
            f'Sharpness: {sharpness:.1f}',
            f'Best: {self.best_sharpness:.1f}',
            f'Frame: {self.frame_count}',
            f'FPS: {self.current_fps:.1f}',
        ]
        
        y_offset = 30
        for line in info_lines:
            cv2.putText(combined, line, (10, y_offset), font, 0.7, (0, 255, 0), 2)
            y_offset += 30
        
        # 对焦状态提示
        if sharpness >= self.best_sharpness * 0.95:
            status = 'GOOD FOCUS'
            color = (0, 255, 0)
        elif sharpness >= self.best_sharpness * 0.8:
            status = 'OK'
            color = (0, 255, 255)
        else:
            status = 'ADJUST FOCUS'
            color = (0, 0, 255)
        
        cv2.putText(combined, status, (10, y_offset + 20), font, 1.0, color, 2)
        
        # 提示按t切换话题
        cv2.putText(combined, "Press 't' to switch topic", (10, combined.shape[0] - 20), 
                   font, 0.5, (150, 150, 150), 1)
        
        return combined

    def create_trend_graph(self, current_sharpness):
        """创建清晰度趋势图"""
        graph = np.ones((self.trend_height, self.trend_width, 3), dtype=np.uint8) * 40
        
        if len(self.sharpness_history) < 2:
            return graph
        
        values = list(self.sharpness_history)
        min_val = min(values)
        max_val = max(values)
        val_range = max_val - min_val if max_val > min_val else 1.0
        
        # 绘制网格
        for i in range(1, 4):
            y = int(self.trend_height * i / 4)
            cv2.line(graph, (0, y), (self.trend_width, y), (60, 60, 60), 1)
        
        # 绘制曲线
        points = []
        for i, val in enumerate(values):
            x = int(self.trend_width * i / (len(values) - 1))
            y = int(self.trend_height * (1 - (val - min_val) / val_range) * 0.9 + self.trend_height * 0.05)
            y = max(5, min(self.trend_height - 5, y))
            points.append((x, y))
        
        for i in range(len(points) - 1):
            cv2.line(graph, points[i], points[i + 1], (100, 200, 255), 2)
        
        # 当前点
        if points:
            cv2.circle(graph, points[-1], 6, (0, 255, 0), -1)
        
        return graph

    def image_callback(self, msg):
        try:
            # 转换图像
            if USE_CV_BRIDGE:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                except Exception:
                    cv_image = ros_image_to_cv2(msg)
            else:
                cv_image = ros_image_to_cv2(msg)
            
            self.frame_count += 1
            self.fps_count += 1
            
            # 计算清晰度
            sharpness, laplacian = self.calculate_sharpness(cv_image)
            
            # 更新历史
            self.sharpness_history.append(sharpness)
            if sharpness > self.best_sharpness:
                self.best_sharpness = sharpness
            
            # 计算FPS
            now = time.time()
            if now - self.fps_start_time >= 1.0:
                self.current_fps = self.fps_count / (now - self.fps_start_time)
                self.fps_count = 0
                self.fps_start_time = now
            
            # GUI显示（仅在非选择模式下）
            if self.gui_mode and not self.selecting_topic:
                if self.init_gui():
                    display = self.create_display(cv_image, laplacian, sharpness)
                    cv2.imshow('Focus Assistant', display)
                    
                    key = cv2.waitKey(1) & 0xFF
                    self.handle_key_input(key)
            
            # 终端输出
            if now - self.last_print_time >= 0.5:
                diff = sharpness - self.best_sharpness
                status = '[OK]' if sharpness >= self.best_sharpness * 0.95 else ''
                self.get_logger().info(
                    f'帧{self.frame_count}: 清晰度={sharpness:.1f}, '
                    f'最佳={self.best_sharpness:.1f} ({diff:+.1f}), '
                    f'FPS={self.current_fps:.1f} {status}'
                )
                self.last_print_time = now
                
        except Exception as e:
            self.get_logger().error(f'处理错误: {e}')
    
    def handle_key_input(self, key):
        """处理键盘输入"""
        if key == ord('q'):
            self.get_logger().info('用户退出')
            rclpy.shutdown()
        elif key == ord('r'):
            if not self.selecting_topic:
                self.sharpness_history.clear()
                self.best_sharpness = 0.0
                self.get_logger().info('已重置')
        elif key == ord('s'):
            # 重新扫描话题
            self.available_topics = scan_image_topics(self)
            if self.available_topics:
                self.selecting_topic = True
                self.get_logger().info(f'发现 {len(self.available_topics)} 个图像话题')
            else:
                self.get_logger().warn('未发现图像话题')
        elif self.selecting_topic:
            # 话题选择模式：数字键1-9
            if ord('1') <= key <= ord('9'):
                idx = key - ord('1')
                if idx < len(self.available_topics):
                    selected_topic = self.available_topics[idx]
                    self.subscribe_to_topic(selected_topic)
                    self.selecting_topic = False
                    self.get_logger().info(f'已选择话题: {selected_topic}')
        elif key == ord('t'):
            # 运行时切换话题
            if not self.selecting_topic:
                self.available_topics = scan_image_topics(self)
                if self.available_topics:
                    self.selecting_topic = True
                    self.get_logger().info(f'切换到话题选择模式，发现 {len(self.available_topics)} 个话题')
                else:
                    self.get_logger().warn('未发现图像话题')


def main():
    parser = argparse.ArgumentParser(description='对焦辅助工具')
    parser.add_argument('--topic', type=str, default=None,
                        help='图像话题 (默认: None，启动时选择)')
    parser.add_argument('--history', type=int, default=200,
                        help='历史记录大小 (默认: 200)')
    parser.add_argument('--no-gui', action='store_true',
                        help='禁用GUI，仅终端输出')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = FocusAssistant(
        topic_name=args.topic,
        history_size=args.history,
        gui_mode=not args.no_gui
    )
    
    try:
        # 主循环：处理话题选择和图像处理
        while rclpy.ok():
            if node.selecting_topic:
                # 话题选择模式
                if node.gui_mode and node.init_gui():
                    selection_display = node.create_topic_selection_display()
                    cv2.imshow('Focus Assistant', selection_display)
                    key = cv2.waitKey(100) & 0xFF
                    node.handle_key_input(key)
                else:
                    # 无GUI模式，列出话题让用户选择
                    if node.available_topics:
                        node.get_logger().info("可用话题:")
                        for i, topic in enumerate(node.available_topics[:9]):
                            node.get_logger().info(f"  {i+1}. {topic}")
                    else:
                        node.get_logger().warn("未发现图像话题，等待...")
                        time.sleep(1)
                        node.available_topics = scan_image_topics(node)
                rclpy.spin_once(node, timeout_sec=0.1)
            else:
                # 正常模式：处理图像
                rclpy.spin_once(node, timeout_sec=0.1)
                
                # 如果GUI模式，需要处理键盘输入（即使没有图像）
                if node.gui_mode and node.init_gui() and not node.selecting_topic:
                    key = cv2.waitKey(1) & 0xFF
                    if key != 255:  # 有按键
                        node.handle_key_input(key)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        if node.gui_initialized:
            cv2.destroyAllWindows()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
