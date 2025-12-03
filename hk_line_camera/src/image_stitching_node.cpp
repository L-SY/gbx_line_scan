#include "hk_line_camera/image_stitching_node.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <chrono>
#include <algorithm>

ImageStitchingNode::ImageStitchingNode()
  : Node("image_stitching_node"),
    stitched_image_(),
    total_frames_processed_(0),
    total_stitches_(0)
{
  // Declare parameters
  this->declare_parameter<std::string>("input_topic", "image_raw");
  this->declare_parameter<std::string>("output_topic", "image_stitched");
  this->declare_parameter<std::string>("frame_id", "camera_frame");
  this->declare_parameter<int>("max_height", 0);              // 0 = unlimited
  this->declare_parameter<int>("max_stitch_count", 0);        // 0 = unlimited
  this->declare_parameter<bool>("publish_periodically", false);
  this->declare_parameter<double>("publish_rate", 1.0);       // Hz
  this->declare_parameter<bool>("reset_on_max_height", true);
  this->declare_parameter<bool>("reset_on_max_count", true);
  
  // Get parameters
  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  max_height_ = this->get_parameter("max_height").as_int();
  max_stitch_count_ = this->get_parameter("max_stitch_count").as_int();
  publish_periodically_ = this->get_parameter("publish_periodically").as_bool();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  reset_on_max_height_ = this->get_parameter("reset_on_max_height").as_bool();
  reset_on_max_count_ = this->get_parameter("reset_on_max_count").as_bool();
  
  stitch_count_ = 0;
  
  // Create subscriber with default QoS (reliable, matches camera node)
  // Using default QoS (reliable, keep_last) which should match the camera node's publisher
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_topic_,
    10,  // QoS depth
    std::bind(&ImageStitchingNode::imageCallback, this, std::placeholders::_1)
  );
  
  RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", image_sub_->get_topic_name());
  
  // Create publisher
  stitched_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
  RCLCPP_INFO(this->get_logger(), "Created publisher on topic: %s", stitched_image_pub_->get_topic_name());
  
  // Create timer for periodic publishing (if enabled)
  if (publish_periodically_) {
    auto period_ms = static_cast<int>(1000.0 / publish_rate_);
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&ImageStitchingNode::timerCallback, this)
    );
  }
  
  // Create reset service
  reset_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/reset_stitching",
    std::bind(&ImageStitchingNode::resetServiceCallback, this, 
              std::placeholders::_1, std::placeholders::_2)
  );
  RCLCPP_INFO(this->get_logger(), "Reset service created: ~/reset_stitching");
  
  RCLCPP_INFO(this->get_logger(), "Image stitching node initialized");
  RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Max height: %d (0 = unlimited)", max_height_);
  RCLCPP_INFO(this->get_logger(), "  Max stitch count: %d (0 = unlimited)", max_stitch_count_);
  RCLCPP_INFO(this->get_logger(), "  Publish periodically: %s", publish_periodically_ ? "true" : "false");
  
  // Print actual topic names after creation
  if (image_sub_) {
    RCLCPP_INFO(this->get_logger(), "  Actual subscribed topic: %s", image_sub_->get_topic_name());
  }
  if (stitched_image_pub_) {
    RCLCPP_INFO(this->get_logger(), "  Actual published topic: %s", stitched_image_pub_->get_topic_name());
  }
}

ImageStitchingNode::~ImageStitchingNode()
{
  RCLCPP_INFO(this->get_logger(), "Image stitching node shutting down");
  RCLCPP_INFO(this->get_logger(), "  Total frames processed: %d", total_frames_processed_);
  RCLCPP_INFO(this->get_logger(), "  Total stitches: %d", total_stitches_);
}

void ImageStitchingNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "First image received! Callback is working.");
  
  std::lock_guard<std::mutex> lock(image_mutex_);
  
  try {
    // Convert ROS image message to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    std::string encoding = msg->encoding;
    
    RCLCPP_INFO(this->get_logger(), "Received image: %dx%d, encoding: %s", 
                 msg->width, msg->height, encoding.c_str());
    
    try {
      // Try to convert with original encoding first
      cv_ptr = cv_bridge::toCvCopy(msg);
    } catch (cv_bridge::Exception& e) {
      // Try MONO8
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      } catch (cv_bridge::Exception& e2) {
        // Try BGR8
        try {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e3) {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s (tried original, MONO8, BGR8)", e3.what());
          return;
        }
      }
    }
    
    total_frames_processed_++;
    RCLCPP_INFO(this->get_logger(), "Successfully converted image to OpenCV Mat: %dx%d, channels: %d, encoding: %s", 
                 cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.channels(), encoding.c_str());
    
    // Stitch the image
    stitchImages(cv_ptr->image);
    
    // Always publish after stitching (if not using periodic publishing)
    // This ensures the first image is published immediately after initialization
    if (!publish_periodically_) {
      publishStitchedImage();
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Skipping immediate publish (using periodic publishing)");
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
  }
}

void ImageStitchingNode::stitchImages(const cv::Mat& new_image)
{
  if (new_image.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty image, skipping");
    return;
  }
  
  // Check if we need to reset
  if (max_height_ > 0 && reset_on_max_height_ && stitched_image_.rows >= max_height_) {
    RCLCPP_INFO(this->get_logger(), "Max height (%d) reached, resetting stitched image", max_height_);
    resetStitchedImage();
  }
  
  if (max_stitch_count_ > 0 && reset_on_max_count_ && stitch_count_ >= max_stitch_count_) {
    RCLCPP_INFO(this->get_logger(), "Max stitch count (%d) reached, resetting stitched image", max_stitch_count_);
    resetStitchedImage();
  }
  
  // Initialize stitched image if empty
  if (stitched_image_.empty()) {
    stitched_image_ = new_image.clone();
    stitch_count_ = 1;
    total_stitches_++;
    RCLCPP_INFO(this->get_logger(), "Initialized stitched image: %dx%d", 
                 stitched_image_.cols, stitched_image_.rows);
    // Note: Don't return here - let it publish in imageCallback after stitchImages returns
    return;
  }
  
  // Check if dimensions match (width must match)
  if (stitched_image_.cols != new_image.cols) {
    RCLCPP_WARN(this->get_logger(), 
                "Width mismatch: stitched=%d, new=%d. Resetting stitched image.",
                stitched_image_.cols, new_image.cols);
    resetStitchedImage();
    stitched_image_ = new_image.clone();
    stitch_count_ = 1;
    total_stitches_++;
    return;
  }
  
  // Check if max height will be exceeded
  if (max_height_ > 0 && (stitched_image_.rows + new_image.rows) > max_height_) {
    if (reset_on_max_height_) {
      // Reset and start new
      resetStitchedImage();
      stitched_image_ = new_image.clone();
      stitch_count_ = 1;
      total_stitches_++;
      return;
    } else {
      // Don't add more, just keep current
      RCLCPP_DEBUG(this->get_logger(), "Max height would be exceeded, skipping stitch");
      return;
    }
  }
  
  // Check if max stitch count will be exceeded
  if (max_stitch_count_ > 0 && stitch_count_ >= max_stitch_count_) {
    if (reset_on_max_count_) {
      // Reset and start new
      resetStitchedImage();
      stitched_image_ = new_image.clone();
      stitch_count_ = 1;
      total_stitches_++;
      return;
    } else {
      // Don't add more, just keep current
      RCLCPP_DEBUG(this->get_logger(), "Max stitch count reached, skipping stitch");
      return;
    }
  }
  
  // Perform vertical stitching (concatenate vertically)
  cv::vconcat(stitched_image_, new_image, stitched_image_);
  stitch_count_++;
  total_stitches_++;
  
  RCLCPP_INFO(this->get_logger(), "Stitched image: %dx%d (count: %d)", 
               stitched_image_.cols, stitched_image_.rows, stitch_count_);
}

void ImageStitchingNode::resetStitchedImage()
{
  stitched_image_.release();
  stitch_count_ = 0;
  RCLCPP_DEBUG(this->get_logger(), "Reset stitched image");
}

void ImageStitchingNode::publishStitchedImage()
{
  if (stitched_image_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Stitched image is empty, skipping publish");
    return;
  }
  
  try {
    // Convert OpenCV Mat to ROS image message
    sensor_msgs::msg::Image img_msg;
    
    // Determine encoding based on image type
    std::string encoding;
    if (stitched_image_.channels() == 1) {
      encoding = sensor_msgs::image_encodings::MONO8;
    } else if (stitched_image_.channels() == 3) {
      encoding = sensor_msgs::image_encodings::BGR8;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported image channels: %d", stitched_image_.channels());
      return;
    }
    
    // Use cv_bridge to convert, which handles step calculation correctly
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = this->now();
    cv_image.header.frame_id = frame_id_;
    cv_image.encoding = encoding;
    cv_image.image = stitched_image_;
    
    // Convert to ROS message - cv_bridge handles step calculation automatically
    img_msg = *(cv_image.toImageMsg());
    
    // Log image message details for debugging
    size_t data_size_bytes = img_msg.data.size();
    size_t expected_size = img_msg.step * img_msg.height;
    
    RCLCPP_INFO(this->get_logger(), 
                "Image message: %dx%d, encoding=%s, step=%u, data_size=%zu bytes, expected_size=%zu bytes", 
                img_msg.width, img_msg.height, img_msg.encoding.c_str(), 
                img_msg.step, data_size_bytes, expected_size);
    
    // Verify data size matches
    if (data_size_bytes != expected_size) {
      RCLCPP_ERROR(this->get_logger(), 
                   "Data size mismatch! This will cause display issues. Expected %zu, got %zu", 
                   expected_size, data_size_bytes);
      // Try to fix by recalculating
      img_msg.data.resize(expected_size);
      if (expected_size > data_size_bytes) {
        RCLCPP_WARN(this->get_logger(), "Expanding data buffer");
      }
    }
    
    // Warn if image is very large (RQT might have trouble)
    if (stitched_image_.rows > 4000 || stitched_image_.cols > 4000) {
      RCLCPP_WARN(this->get_logger(), 
                  "Large image detected (%dx%d). RQT Image View may not display correctly. "
                  "Consider using rviz2 or limiting max_height parameter.", 
                  stitched_image_.cols, stitched_image_.rows);
    }
    
    // Publish (always publish, even if no subscribers - important for debugging)
    stitched_image_pub_->publish(img_msg);
    
    int subscriber_count = stitched_image_pub_->get_subscription_count();
    RCLCPP_INFO(this->get_logger(), 
                "Published stitched image to topic '%s': %dx%d pixels, %d frames stitched, %d subscribers", 
                stitched_image_pub_->get_topic_name(),
                stitched_image_.cols, stitched_image_.rows, stitch_count_, subscriber_count);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error publishing stitched image: %s", e.what());
  }
}

void ImageStitchingNode::timerCallback()
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  publishStitchedImage();
}

void ImageStitchingNode::resetServiceCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Unused parameter
  
  std::lock_guard<std::mutex> lock(image_mutex_);
  
  int previous_count = stitch_count_;
  cv::Size previous_size = stitched_image_.size();
  
  // Reset the stitched image
  resetStitchedImage();
  
  response->success = true;
  response->message = "Stitched image reset successfully. "
                     "Previous size: " + std::to_string(previous_size.width) + "x" + 
                     std::to_string(previous_size.height) + ", " +
                     "Previous count: " + std::to_string(previous_count);
  
  RCLCPP_INFO(this->get_logger(), "Reset service called: %s", response->message.c_str());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageStitchingNode>());
  rclcpp::shutdown();
  return 0;
}

