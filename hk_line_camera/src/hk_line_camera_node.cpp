#include "hk_line_camera/hk_line_camera_node.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <chrono>
#include <cstring>

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
  
  // Camera selection
  this->declare_parameter<int>("camera_index", 0);
  this->declare_parameter<std::string>("frame_id", "camera_frame");
  this->declare_parameter<std::string>("image_topic", "image_raw");
  
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
  
  camera_index_ = this->get_parameter("camera_index").as_int();
  frame_id_ = this->get_parameter("frame_id").as_string();
  std::string image_topic = this->get_parameter("image_topic").as_string();
  
  // Create image publisher (using standard ROS2 publisher to avoid shared_from_this issue)
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);
  
  RCLCPP_INFO(this->get_logger(), "Initializing Hikvision line scan camera...");
  
  if (initializeCamera()) {
    RCLCPP_INFO(this->get_logger(), "Camera initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Image publisher initialized on topic: %s", image_topic.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
    rclcpp::shutdown();
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
  
  if (camera_index_ >= static_cast<int>(stDeviceList.nDeviceNum)) {
    RCLCPP_ERROR(this->get_logger(), "Camera index %d is out of range (max: %d)", 
                 camera_index_, stDeviceList.nDeviceNum - 1);
    MV_CC_Finalize();
    return false;
  }
  
  // Create handle
  nRet = MV_CC_CreateHandle(&camera_handle_, stDeviceList.pDeviceInfo[camera_index_]);
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
  if (stDeviceList.pDeviceInfo[camera_index_]->nTLayerType == MV_GIGE_DEVICE) {
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
  
  if (!camera_initialized_ || image_pub_->get_subscription_count() == 0) {
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
    
    // Create ROS2 image message
    sensor_msgs::msg::Image img_msg;
    img_msg.header.stamp = this->now();
    img_msg.header.frame_id = frame_id_;
    img_msg.width = width;
    img_msg.height = height;
    img_msg.encoding = encoding;
    img_msg.is_bigendian = false;
    img_msg.step = image.step;
    
    size_t data_size = img_msg.step * height;
    img_msg.data.resize(data_size);
    memcpy(img_msg.data.data(), image.data, data_size);
    
    // Publish image
    image_pub_->publish(img_msg);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HkLineCameraNode>());
  rclcpp::shutdown();
  return 0;
}
