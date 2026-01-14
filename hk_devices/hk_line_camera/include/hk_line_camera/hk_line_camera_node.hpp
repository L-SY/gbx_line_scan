#ifndef HK_LINE_CAMERA_NODE_HPP
#define HK_LINE_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "MvCameraControl.h"
#include "hk_line_camera/msg/frame_info.hpp"
#include <memory>
#include <string>
#include <thread>
#include <mutex>

class HkLineCameraNode : public rclcpp::Node
{
public:
  HkLineCameraNode();
  ~HkLineCameraNode();

private:
  // Camera initialization and control
  bool initializeCamera();
  void cleanupCamera();
  bool configureCameraParameters();
  
  // Find camera by IP address (returns device index, -1 if not found)
  int findCameraByIp(MV_CC_DEVICE_INFO_LIST* pDeviceList, const std::string& ip);
  
  // Image callback from SDK
  static void __stdcall imageCallback(unsigned char *pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
  void processImage(unsigned char *pData, MV_FRAME_OUT_INFO_EX* pFrameInfo);
  
  // Parameter setting functions
  bool setFrameTriggerParameters();
  bool setLineTriggerParameters();
  bool setEncoderParameters();
  bool setExposureParameters();
  bool setImageHeight();
  
  // Helper functions
  bool setEnumValue(const std::string& key, int value);
  bool setFloatValue(const std::string& key, float value);
  bool setIntValue(const std::string& key, int64_t value);
  
  // Camera handle
  void* camera_handle_;
  bool camera_initialized_;
  std::mutex image_mutex_;
  
  // ROS2 publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<hk_line_camera::msg::FrameInfo>::SharedPtr frame_info_pub_;
  
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  // Parameters
  // Frame Trigger parameters (FrameBurstStart - 帧触发)
  bool frame_trigger_enabled_;
  int frame_trigger_mode_;
  int frame_trigger_source_;
  int frame_trigger_activation_;
  double frame_trigger_delay_;
  
  // Line Trigger parameters (LineStart - 行触发)
  bool line_trigger_enabled_;
  int line_trigger_mode_;
  int line_trigger_source_;
  int line_trigger_activation_;
  double line_trigger_delay_;
  
  // Encoder parameters (used with line trigger)
  int encoder_selector_;
  int encoder_source_a_;
  int encoder_source_b_;
  int encoder_trigger_mode_;
  int encoder_counter_mode_;
  int64_t encoder_counter_;
  int64_t encoder_counter_max_;
  int64_t encoder_max_reverse_counter_;
  
  // Exposure parameters
  double exposure_time_us_;
  
  // Image control parameters
  int image_height_;  // Maximum image height (0 = no limit)
  
  // Cropping parameters (percentage 0.0-1.0)
  double crop_left_;   // Percentage to crop from left
  double crop_right_;  // Percentage to crop from right
  
  // Camera selection
  int camera_index_;
  std::string camera_ip_;  // IP address for GigE camera (if specified, overrides camera_index)
  std::string frame_id_;
  
  // Frame info publishing option
  bool publish_frame_info_;
};

#endif // HK_LINE_CAMERA_NODE_HPP
