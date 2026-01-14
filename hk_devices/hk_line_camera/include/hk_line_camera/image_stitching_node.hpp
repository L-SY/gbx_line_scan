#ifndef HK_LINE_CAMERA_IMAGE_STITCHING_NODE_HPP
#define HK_LINE_CAMERA_IMAGE_STITCHING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <mutex>
#include <deque>

class ImageStitchingNode : public rclcpp::Node
{
public:
  ImageStitchingNode();
  ~ImageStitchingNode();

private:
  // Image callback from subscribed topic
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  
  // Stitching logic
  void stitchImages(const cv::Mat& new_image);
  void resetStitchedImage();
  void publishStitchedImage();
  
  // Timer callback for periodic publishing (if enabled)
  void timerCallback();
  
  // Service callback for reset
  void resetServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  // ROS2 subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stitched_image_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  
  // Image storage
  cv::Mat stitched_image_;
  std::mutex image_mutex_;
  
  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  
  int max_height_;                    // Maximum height of stitched image (0 = unlimited)
  int stitch_count_;                  // Current number of images stitched
  int max_stitch_count_;              // Maximum number of images to stitch (0 = unlimited)
  bool publish_periodically_;         // Whether to publish periodically
  double publish_rate_;               // Publishing rate in Hz (if periodic)
  bool reset_on_max_height_;          // Reset when max height is reached
  bool reset_on_max_count_;           // Reset when max count is reached
  
  // Statistics
  int total_frames_processed_;
  int total_stitches_;
};

#endif // HK_LINE_CAMERA_IMAGE_STITCHING_NODE_HPP

