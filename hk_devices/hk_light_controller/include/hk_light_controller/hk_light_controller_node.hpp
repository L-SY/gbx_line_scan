#ifndef HK_LIGHT_CONTROLLER_NODE_HPP
#define HK_LIGHT_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "MvCameraControl.h"
#include <string>
#include <memory>
#include <mutex>

class HkLightControllerNode : public rclcpp::Node
{
public:
  HkLightControllerNode();
  ~HkLightControllerNode();
  
  // Public access to initialization status
  bool interface_initialized_;

private:
  // SDK related
  void* interface_handle_;
  bool using_device_handle_;  // True if using device handle instead of interface handle
  std::string target_ip_address_;
  std::mutex control_mutex_;

  // ROS2 publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr light1_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr light2_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr light1_brightness_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr light2_brightness_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trigger_source_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connection_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr light1_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr light1_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr light2_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr light2_current_pub_;

  // ROS2 subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr light1_control_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr light2_control_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr light1_brightness_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr light2_brightness_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_source_sub_;

  // ROS2 services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr light1_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr light2_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_status_service_;

  // Timer for periodic status publishing
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Current state
  bool light1_enabled_;
  bool light2_enabled_;
  int64_t light1_brightness_;
  int64_t light2_brightness_;
  std::string current_trigger_source_;
  float light1_voltage_;
  float light1_current_;
  float light2_voltage_;
  float light2_current_;
  bool voltage_current_warning_logged_;  // Track if we've already warned about missing params

  // Methods
  bool initializeInterface();
  void cleanupInterface();
  bool findInterfaceByIP(const std::string& ip_address, MV_INTERFACE_INFO* pInterfaceInfo);
  bool setLightState(int light_index, bool enabled);
  bool setLightBrightness(int light_index, int64_t brightness);
  bool setTriggerSource(const std::string& trigger_source);
  void publishStatus();
  void updateStatus();

  // Callbacks
  void light1ControlCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void light2ControlCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void light1BrightnessCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void light2BrightnessCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void triggerSourceCallback(const std_msgs::msg::String::SharedPtr msg);
  void light1ServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void light2ServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void getStatusServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // SDK helper methods
  bool setBoolValue(const std::string& key, bool value);
  bool getBoolValue(const std::string& key, bool& value);
  bool setIntValue(const std::string& key, int64_t value);
  bool getIntValue(const std::string& key, int64_t& value);
  bool setEnumValue(const std::string& key, int value);
  bool getEnumValue(const std::string& key, int& value);
  bool getEnumEntrySymbolic(const std::string& key, int value, std::string& symbolic);
  bool getFloatValue(const std::string& key, float& value);
  void logAvailableVoltageCurrentParams();  // Helper to debug parameter names
};

#endif // HK_LIGHT_CONTROLLER_NODE_HPP

