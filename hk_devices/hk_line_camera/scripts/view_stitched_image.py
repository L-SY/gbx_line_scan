#!/usr/bin/env python3
"""
Simple script to view stitched images from /image_stitched topic
This script can handle large images better than RQT Image View
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse


class ImageViewer(Node):
    def __init__(self, topic_name, max_display_height=1080, scale_to_fit=True):
        super().__init__('stitched_image_viewer')
        self.bridge = CvBridge()
        self.max_display_height = max_display_height
        self.scale_to_fit = scale_to_fit
        
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10)
        
        self.get_logger().info(f'Subscribing to topic: {topic_name}')
        self.get_logger().info(f'Max display height: {max_display_height}')
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            
            self.frame_count += 1
            height, width = cv_image.shape[:2]
            
            self.get_logger().info(f'Received image #{self.frame_count}: {width}x{height} pixels')
            
            # Scale down if image is too large
            display_image = cv_image
            if self.scale_to_fit and height > self.max_display_height:
                scale = self.max_display_height / height
                new_width = int(width * scale)
                new_height = self.max_display_height
                display_image = cv2.resize(cv_image, (new_width, new_height), interpolation=cv2.INTER_AREA)
                self.get_logger().info(f'Scaled for display: {new_width}x{new_height} (scale: {scale:.2f})')
            
            # Display image
            cv2.imshow('Stitched Image (Press Q to quit)', display_image)
            
            # Show original dimensions in window title
            win_name = f'Stitched Image - Original: {width}x{height} - Frame: {self.frame_count}'
            cv2.setWindowTitle('Stitched Image (Press Q to quit)', win_name)
            
            # Wait for key press (non-blocking)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                self.get_logger().info('Quit requested by user')
                rclpy.shutdown()
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
            import traceback
            traceback.print_exc()


def main(args=None):
    parser = argparse.ArgumentParser(description='View stitched images from ROS2 topic')
    parser.add_argument('--topic', type=str, default='/image_stitched',
                       help='Image topic to subscribe to (default: /image_stitched)')
    parser.add_argument('--max-height', type=int, default=1080,
                       help='Maximum display height in pixels (default: 1080)')
    parser.add_argument('--no-scale', action='store_true',
                       help='Do not scale large images (may cause display issues)')
    
    args = parser.parse_args()
    
    rclpy.init()
    viewer = ImageViewer(args.topic, 
                        max_display_height=args.max_height,
                        scale_to_fit=not args.no_scale)
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        viewer.get_logger().info('Interrupted by user')
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

