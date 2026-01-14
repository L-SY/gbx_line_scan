#include "hk_line_camera/hk_line_camera_node.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <algorithm>

HkLineCameraNode::HkLineCameraNode()
  : Node("hk_line_camera_node"),
    camera_handle_(nullptr),
    camera_initialized_(false)
{
  // Declare parameters
  // Frame Trigger parameters (FrameBurstStart - 帧触发)
  this->declare_parameter<bool>("frame_trigger_enabled", false);
  this->declare_parameter<int>("frame_trigger_mode", 1);      // 1 = On, 0 = Off
  this->declare_parameter<int>("frame_trigger_source", 1);    // 1 = Line1
  this->declare_parameter<int>("frame_trigger_activation", 0); // 0 = RisingEdge, 1 = FallingEdge
  this->declare_parameter<double>("frame_trigger_delay", 0.0);
  
  // Line Trigger parameters (LineStart - 行触发)
  this->declare_parameter<bool>("line_trigger_enabled", true);
  this->declare_parameter<int>("line_trigger_mode", 1);      // 1 = On, 0 = Off
  this->declare_parameter<int>("line_trigger_source", 6);    // 6 = EncoderModuleOut
  this->declare_parameter<int>("line_trigger_activation", 0); // 0 = RisingEdge, 1 = FallingEdge
  this->declare_parameter<double>("line_trigger_delay", 0.0);
  
  // Encoder parameters (used with line trigger)
  this->declare_parameter<int>("encoder_selector", 0);
  this->declare_parameter<int>("encoder_source_a", 3);  // Line 3
  this->declare_parameter<int>("encoder_source_b", 0);  // Line 0
  this->declare_parameter<int>("encoder_trigger_mode", 0); // 0 = AnyDirection
  this->declare_parameter<int>("encoder_counter_mode", 0); // 0 = IgnoreDirection
  this->declare_parameter<int64_t>("encoder_counter", 0);
  this->declare_parameter<int64_t>("encoder_counter_max", 10000);
  this->declare_parameter<int64_t>("encoder_max_reverse_counter", 0);
  
  // Exposure parameters
  this->declare_parameter<double>("exposure_time_us", 300.0);
  
  // Image control parameters
  this->declare_parameter<int>("image_height", 0);  // 0 = no limit
  
  // Cropping parameters (percentage 0.0-1.0)
  this->declare_parameter<double>("crop_left", 0.0);
  this->declare_parameter<double>("crop_right", 0.0);
  this->declare_parameter<bool>("publish_debug_image", false);
  
  // Camera selection
  this->declare_parameter<int>("camera_index", 0);
  this->declare_parameter<std::string>("camera_ip", "");  // IP address for GigE camera (if specified, overrides camera_index)
  this->declare_parameter<std::string>("frame_id", "camera_frame");
  this->declare_parameter<std::string>("image_topic", "image_raw");
  
  // Frame info publishing
  this->declare_parameter<bool>("publish_frame_info", true);
  this->declare_parameter<std::string>("frame_info_topic", "frame_info");
  
  // Get parameters
  // Frame trigger
  frame_trigger_enabled_ = this->get_parameter("frame_trigger_enabled").as_bool();
  frame_trigger_mode_ = this->get_parameter("frame_trigger_mode").as_int();
  frame_trigger_source_ = this->get_parameter("frame_trigger_source").as_int();
  frame_trigger_activation_ = this->get_parameter("frame_trigger_activation").as_int();
  frame_trigger_delay_ = this->get_parameter("frame_trigger_delay").as_double();
  
  // Line trigger
  line_trigger_enabled_ = this->get_parameter("line_trigger_enabled").as_bool();
  line_trigger_mode_ = this->get_parameter("line_trigger_mode").as_int();
  line_trigger_source_ = this->get_parameter("line_trigger_source").as_int();
  line_trigger_activation_ = this->get_parameter("line_trigger_activation").as_int();
  line_trigger_delay_ = this->get_parameter("line_trigger_delay").as_double();
  
  // Encoder
  encoder_selector_ = this->get_parameter("encoder_selector").as_int();
  encoder_source_a_ = this->get_parameter("encoder_source_a").as_int();
  encoder_source_b_ = this->get_parameter("encoder_source_b").as_int();
  encoder_trigger_mode_ = this->get_parameter("encoder_trigger_mode").as_int();
  encoder_counter_mode_ = this->get_parameter("encoder_counter_mode").as_int();
  encoder_counter_ = this->get_parameter("encoder_counter").as_int();
  encoder_counter_max_ = this->get_parameter("encoder_counter_max").as_int();
  encoder_max_reverse_counter_ = this->get_parameter("encoder_max_reverse_counter").as_int();
  
  exposure_time_us_ = this->get_parameter("exposure_time_us").as_double();
  
  image_height_ = this->get_parameter("image_height").as_int();
  
  // Get cropping parameters
  crop_left_ = this->get_parameter("crop_left").as_double();
  crop_right_ = this->get_parameter("crop_right").as_double();
  publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
  
  camera_index_ = this->get_parameter("camera_index").as_int();
  camera_ip_ = this->get_parameter("camera_ip").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  std::string image_topic = this->get_parameter("image_topic").as_string();
  
  // Frame info publishing
  publish_frame_info_ = this->get_parameter("publish_frame_info").as_bool();
  std::string frame_info_topic = this->get_parameter("frame_info_topic").as_string();
  
  // Create publishers (using standard ROS2 publisher to avoid shared_from_this issue)
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);
  {
    std::string crop_topic = image_topic + "_crop";
    cropped_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(crop_topic, 10);
    RCLCPP_INFO(this->get_logger(), "Cropped image publisher initialized on topic: %s", crop_topic.c_str());
  }
  
  // Create frame info publisher if enabled
  if (publish_frame_info_) {
    frame_info_pub_ = this->create_publisher<hk_line_camera::msg::FrameInfo>(frame_info_topic, 10);
    RCLCPP_INFO(this->get_logger(), "Frame info publisher initialized on topic: %s", frame_info_topic.c_str());
  }
  
  // Create debug image publisher on image_topic_debug
  {
    std::string debug_topic = image_topic + "_debug";
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_topic, 10);
    RCLCPP_INFO(this->get_logger(), "Debug image publisher initialized on topic: %s", debug_topic.c_str());
  }
  
  RCLCPP_INFO(this->get_logger(), "Initializing Hikvision line scan camera...");
  
  if (initializeCamera()) {
    RCLCPP_INFO(this->get_logger(), "Camera initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Image publisher initialized on topic: %s", image_topic.c_str());
    if (image_height_ > 0) {
      RCLCPP_INFO(this->get_logger(), "Image height configured: %d pixels (set via camera SDK)", image_height_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Image height: using camera default");
    }
    if (crop_left_ > 0.0 || crop_right_ > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Image crop: left=%.1f%%, right=%.1f%%", crop_left_ * 100, crop_right_ * 100);
    }
    if (publish_debug_image_) {
      RCLCPP_INFO(this->get_logger(), "Debug image publishing enabled");
    }
    
    // Register parameter callback for dynamic reconfigure
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & param : parameters) {
          if (param.get_name() == "crop_left") {
            double val = param.as_double();
            if (val >= 0.0 && val < 1.0 && (val + crop_right_) < 1.0) {
              crop_left_ = val;
              RCLCPP_INFO(this->get_logger(), "Updated crop_left: %.1f%%", crop_left_ * 100);
            } else {
              result.successful = false;
              result.reason = "crop_left must be in [0, 1) and crop_left + crop_right < 1";
            }
          } else if (param.get_name() == "crop_right") {
            double val = param.as_double();
            if (val >= 0.0 && val < 1.0 && (crop_left_ + val) < 1.0) {
              crop_right_ = val;
              RCLCPP_INFO(this->get_logger(), "Updated crop_right: %.1f%%", crop_right_ * 100);
            } else {
              result.successful = false;
              result.reason = "crop_right must be in [0, 1) and crop_left + crop_right < 1";
            }
          } else if (param.get_name() == "publish_debug_image") {
            bool val = param.as_bool();
            publish_debug_image_ = val;
            RCLCPP_INFO(this->get_logger(), "Updated publish_debug_image: %s", val ? "true" : "false");
          }
        }
        return result;
      });
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
    throw std::runtime_error("Failed to initialize camera");
  }
}

HkLineCameraNode::~HkLineCameraNode()
{
  cleanupCamera();
}

bool HkLineCameraNode::initializeCamera()
{
  int nRet = MV_OK;
  
  // Initialize SDK
  nRet = MV_CC_Initialize();
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Initialize SDK failed! nRet [0x%x]", nRet);
    return false;
  }
  
  // Enumerate devices
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | 
                           MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Enum Devices failed! nRet [0x%x]", nRet);
    MV_CC_Finalize();
    return false;
  }
  
  if (stDeviceList.nDeviceNum == 0) {
    RCLCPP_ERROR(this->get_logger(), "No devices found!");
    MV_CC_Finalize();
    return false;
  }
  
  RCLCPP_INFO(this->get_logger(), "Found %d device(s)", stDeviceList.nDeviceNum);
  
  // Print all found devices
  for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
    MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
    if (pDeviceInfo && pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
      int nIp1 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
      int nIp2 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
      int nIp3 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
      int nIp4 = (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
      RCLCPP_INFO(this->get_logger(), "  [%d] GigE Camera: %d.%d.%d.%d (%s)", 
                  i, nIp1, nIp2, nIp3, nIp4, 
                  pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
  }
  
  // Determine which camera to use
  int selected_index = camera_index_;
  if (!camera_ip_.empty()) {
    // Use IP address to find camera
    int ip_index = findCameraByIp(&stDeviceList, camera_ip_);
    if (ip_index >= 0) {
      selected_index = ip_index;
      RCLCPP_INFO(this->get_logger(), "Found camera by IP %s at index %d", camera_ip_.c_str(), selected_index);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Camera with IP %s not found!", camera_ip_.c_str());
      MV_CC_Finalize();
      return false;
    }
  } else {
    // Use camera index
    if (camera_index_ >= static_cast<int>(stDeviceList.nDeviceNum)) {
      RCLCPP_ERROR(this->get_logger(), "Camera index %d is out of range (max: %d)", 
                   camera_index_, stDeviceList.nDeviceNum - 1);
      MV_CC_Finalize();
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Using camera at index %d", selected_index);
  }
  
  // Create handle
  nRet = MV_CC_CreateHandle(&camera_handle_, stDeviceList.pDeviceInfo[selected_index]);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Create Handle failed! nRet [0x%x]", nRet);
    MV_CC_Finalize();
    return false;
  }
  
  // Open device
  nRet = MV_CC_OpenDevice(camera_handle_);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Open Device failed! nRet [0x%x]", nRet);
    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
    MV_CC_Finalize();
    return false;
  }
  
  // Set optimal packet size for GigE cameras
  if (stDeviceList.pDeviceInfo[selected_index]->nTLayerType == MV_GIGE_DEVICE) {
    int nPacketSize = MV_CC_GetOptimalPacketSize(camera_handle_);
    if (nPacketSize > 0) {
      nRet = MV_CC_SetIntValueEx(camera_handle_, "GevSCPSPacketSize", nPacketSize);
      if (nRet != MV_OK) {
        RCLCPP_WARN(this->get_logger(), "Set Packet Size failed nRet [0x%x]", nRet);
      } else {
        RCLCPP_INFO(this->get_logger(), "Set optimal packet size: %d", nPacketSize);
      }
    }
  }
  
  // Configure camera parameters
  if (!configureCameraParameters()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure camera parameters");
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
    MV_CC_Finalize();
    return false;
  }
  
  // Register image callback
  nRet = MV_CC_RegisterImageCallBackEx(camera_handle_, imageCallback, this);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Register Image CallBack failed! nRet [0x%x]", nRet);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
    MV_CC_Finalize();
    return false;
  }
  
  // Start grabbing
  nRet = MV_CC_StartGrabbing(camera_handle_);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Start Grabbing failed! nRet [0x%x]", nRet);
    MV_CC_RegisterImageCallBackEx(camera_handle_, NULL, NULL);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
    MV_CC_Finalize();
    return false;
  }
  
  camera_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "Camera started grabbing");
  
  return true;
}

void HkLineCameraNode::cleanupCamera()
{
  if (camera_handle_ != nullptr) {
    if (camera_initialized_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_RegisterImageCallBackEx(camera_handle_, NULL, NULL);
    }
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
  }
  MV_CC_Finalize();
  RCLCPP_INFO(this->get_logger(), "Camera cleaned up");
}

bool HkLineCameraNode::configureCameraParameters()
{
  RCLCPP_INFO(this->get_logger(), "Configuring camera parameters...");
  
  // Configure frame trigger if enabled
  if (frame_trigger_enabled_) {
    if (!setFrameTriggerParameters()) {
      return false;
    }
  }
  
  // Configure line trigger if enabled
  if (line_trigger_enabled_) {
    if (!setLineTriggerParameters()) {
      return false;
    }
    
    // Set encoder parameters for line trigger
    if (!setEncoderParameters()) {
      return false;
    }
  }
  
  if (!frame_trigger_enabled_ && !line_trigger_enabled_) {
    RCLCPP_WARN(this->get_logger(), "Both frame and line triggers are disabled - camera will run in free-run mode");
  }
  
  if (!setExposureParameters()) {
    return false;
  }
  
  // Set image height if specified
  if (!setImageHeight()) {
    RCLCPP_WARN(this->get_logger(), "Failed to set image height, continuing anyway");
  }
  
  RCLCPP_INFO(this->get_logger(), "Camera parameters configured successfully");
  return true;
}

bool HkLineCameraNode::setFrameTriggerParameters()
{
  RCLCPP_INFO(this->get_logger(), "Setting frame trigger parameters (帧触发)...");
  
  // Set Trigger Selector to FrameBurstStart (6)
  if (!setEnumValue("TriggerSelector", 6)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set TriggerSelector to FrameBurstStart");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "TriggerSelector set to FrameBurstStart (6)");
  
  // Set Trigger Mode
  if (!setEnumValue("TriggerMode", frame_trigger_mode_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set frame TriggerMode");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Frame TriggerMode set to %d", frame_trigger_mode_);
  
  // Set Trigger Source (typically Line0 or Line1)
  if (!setEnumValue("TriggerSource", frame_trigger_source_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set frame TriggerSource");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Frame TriggerSource set to Line%d", frame_trigger_source_);
  
  // Set Trigger Activation
  if (!setEnumValue("TriggerActivation", frame_trigger_activation_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set frame TriggerActivation, may not be supported");
  } else {
    RCLCPP_INFO(this->get_logger(), "Frame TriggerActivation set to %d", frame_trigger_activation_);
  }
  
  // Set Trigger Delay
  if (!setFloatValue("TriggerDelay", static_cast<float>(frame_trigger_delay_))) {
    RCLCPP_WARN(this->get_logger(), "Failed to set frame TriggerDelay, may not be supported");
  } else {
    RCLCPP_INFO(this->get_logger(), "Frame TriggerDelay set to %.4f", frame_trigger_delay_);
  }
  
  return true;
}

bool HkLineCameraNode::setLineTriggerParameters()
{
  RCLCPP_INFO(this->get_logger(), "Setting line trigger parameters (行触发)...");
  
  // Set Trigger Selector to LineStart (9)
  if (!setEnumValue("TriggerSelector", 9)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set TriggerSelector to LineStart");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "TriggerSelector set to LineStart (9)");
  
  // Set Trigger Mode
  if (!setEnumValue("TriggerMode", line_trigger_mode_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set line TriggerMode");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Line TriggerMode set to %d", line_trigger_mode_);
  
  // Set Trigger Source (typically EncoderModuleOut)
  if (!setEnumValue("TriggerSource", line_trigger_source_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set line TriggerSource");
    return false;
  }
  if (line_trigger_source_ == 6) {
    RCLCPP_INFO(this->get_logger(), "Line TriggerSource set to EncoderModuleOut (6)");
  } else {
    RCLCPP_INFO(this->get_logger(), "Line TriggerSource set to %d", line_trigger_source_);
  }
  
  // Set Trigger Activation
  if (!setEnumValue("TriggerActivation", line_trigger_activation_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set line TriggerActivation, may not be supported");
  } else {
    RCLCPP_INFO(this->get_logger(), "Line TriggerActivation set to %d", line_trigger_activation_);
  }
  
  // Set Trigger Delay
  if (!setFloatValue("TriggerDelay", static_cast<float>(line_trigger_delay_))) {
    RCLCPP_WARN(this->get_logger(), "Failed to set line TriggerDelay, may not be supported");
  } else {
    RCLCPP_INFO(this->get_logger(), "Line TriggerDelay set to %.4f", line_trigger_delay_);
  }
  
  return true;
}

bool HkLineCameraNode::setEncoderParameters()
{
  RCLCPP_INFO(this->get_logger(), "Setting encoder parameters...");
  
  // Set Encoder Selector
  if (!setEnumValue("EncoderSelector", encoder_selector_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set EncoderSelector, encoder may not be supported");
    return true; // Encoder is optional
  }
  RCLCPP_INFO(this->get_logger(), "EncoderSelector set to %d", encoder_selector_);
  
  // Set Encoder Source A
  if (!setEnumValue("EncoderSourceA", encoder_source_a_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set EncoderSourceA");
  } else {
    RCLCPP_INFO(this->get_logger(), "EncoderSourceA set to %d", encoder_source_a_);
  }
  
  // Set Encoder Source B
  if (!setEnumValue("EncoderSourceB", encoder_source_b_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set EncoderSourceB");
  } else {
    RCLCPP_INFO(this->get_logger(), "EncoderSourceB set to %d", encoder_source_b_);
  }
  
  // Set Encoder Trigger Mode
  if (!setEnumValue("EncoderTriggerMode", encoder_trigger_mode_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set EncoderTriggerMode");
  } else {
    RCLCPP_INFO(this->get_logger(), "EncoderTriggerMode set to %d", encoder_trigger_mode_);
  }
  
  // Set Encoder Counter Mode
  if (!setEnumValue("EncoderCounterMode", encoder_counter_mode_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set EncoderCounterMode");
  } else {
    RCLCPP_INFO(this->get_logger(), "EncoderCounterMode set to %d", encoder_counter_mode_);
  }
  
  // Set Encoder Counter
  if (!setIntValue("EncoderCounter", encoder_counter_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set EncoderCounter");
  } else {
    RCLCPP_INFO(this->get_logger(), "EncoderCounter set to %ld", encoder_counter_);
  }
  
  // Set Encoder Counter Max
  if (!setIntValue("EncoderCounterMax", encoder_counter_max_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set EncoderCounterMax");
  } else {
    RCLCPP_INFO(this->get_logger(), "EncoderCounterMax set to %ld", encoder_counter_max_);
  }
  
  // Set Encoder Max Reverse Counter
  if (!setIntValue("EncoderMaxReverseCounter", encoder_max_reverse_counter_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to set EncoderMaxReverseCounter");
  } else {
    RCLCPP_INFO(this->get_logger(), "EncoderMaxReverseCounter set to %ld", encoder_max_reverse_counter_);
  }
  
  return true;
}

bool HkLineCameraNode::setExposureParameters()
{
  RCLCPP_INFO(this->get_logger(), "Setting exposure parameters...");
  
  // Set Exposure Time (in microseconds)
  if (!setFloatValue("ExposureTime", static_cast<float>(exposure_time_us_))) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set ExposureTime");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "ExposureTime set to %.4f us", exposure_time_us_);
  
  return true;
}

bool HkLineCameraNode::setImageHeight()
{
  if (image_height_ <= 0) {
    // No height limit specified
    return true;
  }
  
  RCLCPP_INFO(this->get_logger(), "Setting image height to %d pixels...", image_height_);
  
  // Get Height parameter info to check range and increment
  MVCC_INTVALUE_EX stHeight = {0};
  int nRet = MV_CC_GetIntValueEx(camera_handle_, "Height", &stHeight);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), 
                "Height parameter: current=%ld, min=%ld, max=%ld, inc=%ld",
                stHeight.nCurValue, stHeight.nMin, stHeight.nMax, stHeight.nInc);
    
    // Check if value is within range
    if (image_height_ < static_cast<int>(stHeight.nMin) || 
        image_height_ > static_cast<int>(stHeight.nMax)) {
      RCLCPP_WARN(this->get_logger(), 
                  "Requested height %d is out of range [%ld, %ld]. Using closest valid value.",
                  image_height_, stHeight.nMin, stHeight.nMax);
      image_height_ = std::max(static_cast<int>(stHeight.nMin), 
                               std::min(static_cast<int>(stHeight.nMax), image_height_));
    }
    
    // Adjust to be a multiple of increment if needed
    if (stHeight.nInc > 0 && (image_height_ % static_cast<int>(stHeight.nInc)) != 0) {
      int adjusted = ((image_height_ / static_cast<int>(stHeight.nInc)) * static_cast<int>(stHeight.nInc));
      RCLCPP_WARN(this->get_logger(), 
                  "Height %d is not a multiple of increment %ld. Adjusting to %d.",
                  image_height_, stHeight.nInc, adjusted);
      image_height_ = adjusted;
    }
    
    // Set Height parameter
    if (setIntValue("Height", image_height_)) {
      RCLCPP_INFO(this->get_logger(), "Image height set to %d pixels", image_height_);
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set Height parameter");
      return false;
    }
  } else {
    RCLCPP_WARN(this->get_logger(), 
                "Height parameter not available (nRet=0x%x). Camera may not support setting image height.", nRet);
    return false;
  }
}

int HkLineCameraNode::findCameraByIp(MV_CC_DEVICE_INFO_LIST* pDeviceList, const std::string& ip)
{
  // Parse target IP address
  int target_ip[4];
  if (sscanf(ip.c_str(), "%d.%d.%d.%d", &target_ip[0], &target_ip[1], &target_ip[2], &target_ip[3]) != 4) {
    RCLCPP_ERROR(this->get_logger(), "Invalid IP address format: %s", ip.c_str());
    return -1;
  }
  
  unsigned int target_ip_value = (target_ip[0] << 24) | (target_ip[1] << 16) | (target_ip[2] << 8) | target_ip[3];
  
  for (unsigned int i = 0; i < pDeviceList->nDeviceNum; i++) {
    MV_CC_DEVICE_INFO* pDeviceInfo = pDeviceList->pDeviceInfo[i];
    if (pDeviceInfo && pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
      if (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp == target_ip_value) {
        return static_cast<int>(i);
      }
    }
  }
  
  return -1;  // Not found
}

bool HkLineCameraNode::setEnumValue(const std::string& key, int value)
{
  int nRet = MV_CC_SetEnumValue(camera_handle_, key.c_str(), value);
  if (MV_OK != nRet) {
    return false;
  }
  return true;
}

bool HkLineCameraNode::setFloatValue(const std::string& key, float value)
{
  int nRet = MV_CC_SetFloatValue(camera_handle_, key.c_str(), value);
  if (MV_OK != nRet) {
    return false;
  }
  return true;
}

bool HkLineCameraNode::setIntValue(const std::string& key, int64_t value)
{
  int nRet = MV_CC_SetIntValueEx(camera_handle_, key.c_str(), value);
  if (MV_OK != nRet) {
    return false;
  }
  return true;
}

void __stdcall HkLineCameraNode::imageCallback(unsigned char *pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
  if (pUser == nullptr || pData == nullptr || pFrameInfo == nullptr) {
    return;
  }
  
  HkLineCameraNode* node = static_cast<HkLineCameraNode*>(pUser);
  node->processImage(pData, pFrameInfo);
}

void HkLineCameraNode::processImage(unsigned char *pData, MV_FRAME_OUT_INFO_EX* pFrameInfo)
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  
  // Check if we have any subscribers for image output
  bool has_image_subscribers =
    (image_pub_ && image_pub_->get_subscription_count() > 0) ||
    (cropped_image_pub_ && cropped_image_pub_->get_subscription_count() > 0) ||
    (publish_debug_image_ && debug_image_pub_ && debug_image_pub_->get_subscription_count() > 0);
  bool has_frame_info_subscribers = publish_frame_info_ && frame_info_pub_ && 
                                     frame_info_pub_->get_subscription_count() > 0;
  
  if (!camera_initialized_ || (!has_image_subscribers && !has_frame_info_subscribers)) {
    return;
  }
  
  // Get current ROS timestamp for synchronization
  auto ros_stamp = this->now();
  
  // Publish frame info if enabled and has subscribers
  if (has_frame_info_subscribers) {
    hk_line_camera::msg::FrameInfo frame_info_msg;
    
    // Header
    frame_info_msg.header.stamp = ros_stamp;
    frame_info_msg.header.frame_id = frame_id_;
    
    // Frame number and counter
    frame_info_msg.frame_num = pFrameInfo->nFrameNum;
    frame_info_msg.frame_counter = pFrameInfo->nFrameCounter;
    frame_info_msg.trigger_index = pFrameInfo->nTriggerIndex;
    
    // Encoder info (line trigger related)
    frame_info_msg.first_line_encoder = pFrameInfo->nFirstLineEncoderCount;
    frame_info_msg.last_line_encoder = pFrameInfo->nLastLineEncoderCount;
    frame_info_msg.encoder_diff = (pFrameInfo->nLastLineEncoderCount >= pFrameInfo->nFirstLineEncoderCount) ?
                                   (pFrameInfo->nLastLineEncoderCount - pFrameInfo->nFirstLineEncoderCount) :
                                   (pFrameInfo->nFirstLineEncoderCount - pFrameInfo->nLastLineEncoderCount);
    
    // IO status
    frame_info_msg.input_status = pFrameInfo->nInput;
    frame_info_msg.output_status = pFrameInfo->nOutput;
    
    // Image parameters (use extended dimensions for line scan cameras)
    frame_info_msg.width = pFrameInfo->nExtendWidth > 0 ? pFrameInfo->nExtendWidth : pFrameInfo->nWidth;
    frame_info_msg.height = pFrameInfo->nExtendHeight > 0 ? pFrameInfo->nExtendHeight : pFrameInfo->nHeight;
    frame_info_msg.exposure_time = pFrameInfo->fExposureTime;
    frame_info_msg.gain = pFrameInfo->fGain;
    
    // Device timestamp
    frame_info_msg.dev_timestamp_high = pFrameInfo->nDevTimeStampHigh;
    frame_info_msg.dev_timestamp_low = pFrameInfo->nDevTimeStampLow;
    
    // Lost packet info
    frame_info_msg.lost_packet = pFrameInfo->nLostPacket;
    
    frame_info_pub_->publish(frame_info_msg);
  }
  
  // Skip image processing if no image subscribers
  if (!has_image_subscribers) {
    return;
  }
  
  try {
    // Get image dimensions (use extended dimensions for line scan cameras)
    int width = pFrameInfo->nExtendWidth > 0 ? pFrameInfo->nExtendWidth : pFrameInfo->nWidth;
    int height = pFrameInfo->nExtendHeight > 0 ? pFrameInfo->nExtendHeight : pFrameInfo->nHeight;
    int pixel_format = pFrameInfo->enPixelType;
    
    // Convert pixel format to OpenCV format
    cv::Mat image;
    std::string encoding;
    
    switch (pixel_format) {
      case PixelType_Gvsp_Mono8:
        // For line scan cameras, height is typically 1 or small
        image = cv::Mat(height, width, CV_8UC1, pData);
        encoding = sensor_msgs::image_encodings::MONO8;
        break;
        
      case PixelType_Gvsp_Mono10:
      case PixelType_Gvsp_Mono10_Packed:
      case PixelType_Gvsp_Mono12:
      case PixelType_Gvsp_Mono12_Packed:
        // Convert 10/12 bit to 8 bit
        {
          cv::Mat raw_image(height, width, CV_16UC1, pData);
          cv::Mat mono8_image;
          raw_image.convertTo(mono8_image, CV_8UC1, 1.0/16.0); // Scale down from 12-bit
          image = mono8_image;
          encoding = sensor_msgs::image_encodings::MONO8;
        }
        break;
        
      case PixelType_Gvsp_RGB8_Packed:
        image = cv::Mat(height, width, CV_8UC3, pData);
        encoding = sensor_msgs::image_encodings::RGB8;
        break;
        
      case PixelType_Gvsp_BGR8_Packed:
        image = cv::Mat(height, width, CV_8UC3, pData);
        encoding = sensor_msgs::image_encodings::BGR8;
        break;
        
      default:
        RCLCPP_WARN(this->get_logger(), "Unsupported pixel format: %d, trying MONO8", pixel_format);
        // Try to create as MONO8, may need pixel format conversion
        image = cv::Mat(height, width, CV_8UC1, pData);
        encoding = sensor_msgs::image_encodings::MONO8;
        break;
    }
    
    // Calculate crop region in pixels (based on original image)
    int img_width = image.cols;
    int img_height = image.rows;
    int crop_x = static_cast<int>(crop_left_ * img_width);
    int crop_w = img_width - static_cast<int>((crop_left_ + crop_right_) * img_width);
    
    // Ensure valid crop region
    crop_x = std::max(0, std::min(crop_x, img_width - 1));
    crop_w = std::max(1, std::min(crop_w, img_width - crop_x));
    
    // Publish debug image with red crop lines (if enabled). This is ALWAYS based on the original image.
    if (publish_debug_image_ && debug_image_pub_ && debug_image_pub_->get_subscription_count() > 0) {
      cv::Mat debug_image;
      // Convert to color if grayscale
      if (image.channels() == 1) {
        cv::cvtColor(image, debug_image, cv::COLOR_GRAY2BGR);
      } else {
        debug_image = image.clone();
      }
      
      // Draw red crop lines
      cv::Scalar red(0, 0, 255);  // BGR format
      int line_thickness = 10;
      
      // Left line
      cv::line(debug_image, cv::Point(crop_x, 0), cv::Point(crop_x, img_height), red, line_thickness);
      // Right line
      cv::line(debug_image, cv::Point(crop_x + crop_w, 0), cv::Point(crop_x + crop_w, img_height), red, line_thickness);
      
      // Publish debug image
      cv_bridge::CvImage debug_cv_image;
      debug_cv_image.header.stamp = ros_stamp;
      debug_cv_image.header.frame_id = frame_id_;
      debug_cv_image.encoding = sensor_msgs::image_encodings::BGR8;
      debug_cv_image.image = debug_image;
      debug_image_pub_->publish(*(debug_cv_image.toImageMsg()));
    }
    
    // Publish cropped image on {image_topic}_crop (used for stitching)
    if (cropped_image_pub_ && cropped_image_pub_->get_subscription_count() > 0) {
      cv::Mat cropped = image;
      if (crop_left_ > 0.0 || crop_right_ > 0.0) {
        cv::Rect crop_roi(crop_x, 0, crop_w, image.rows);
        cropped = image(crop_roi).clone();
      } else if (!image.isContinuous()) {
        // Ensure safe copy for ROS message
        cropped = image.clone();
      }

      sensor_msgs::msg::Image crop_msg;
      crop_msg.header.stamp = ros_stamp;
      crop_msg.header.frame_id = frame_id_;
      crop_msg.width = cropped.cols;
      crop_msg.height = cropped.rows;
      crop_msg.encoding = encoding;
      crop_msg.is_bigendian = false;
      crop_msg.step = cropped.step;

      size_t crop_size = crop_msg.step * crop_msg.height;
      crop_msg.data.resize(crop_size);
      memcpy(crop_msg.data.data(), cropped.data, crop_size);

      cropped_image_pub_->publish(crop_msg);
    }

    // Publish original (uncropped) image on image_topic
    if (image_pub_ && image_pub_->get_subscription_count() > 0) {
      sensor_msgs::msg::Image img_msg;
      img_msg.header.stamp = ros_stamp;  // Use same timestamp as frame_info for synchronization
      img_msg.header.frame_id = frame_id_;
      img_msg.width = image.cols;
      img_msg.height = image.rows;
      img_msg.encoding = encoding;
      img_msg.is_bigendian = false;
      img_msg.step = image.step;

      size_t data_size = img_msg.step * img_msg.height;
      img_msg.data.resize(data_size);
      memcpy(img_msg.data.data(), image.data, data_size);

      image_pub_->publish(img_msg);
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<HkLineCameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("hk_line_camera"), "Node initialization failed: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
