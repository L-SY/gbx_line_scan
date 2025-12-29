#ifndef LIGHT_CONTROLLER_GUI_HPP
#define LIGHT_CONTROLLER_GUI_HPP

#include <QMainWindow>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QComboBox>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

class LightControllerGUI : public QMainWindow
{
  Q_OBJECT

public:
  explicit LightControllerGUI(QWidget *parent = nullptr);
  ~LightControllerGUI();

private slots:
  void onLight1Toggle();
  void onLight2Toggle();
  void onLight1BrightnessChanged(int value);
  void onLight2BrightnessChanged(int value);
  void onTriggerSourceChanged(int index);
  void onRefreshStatus();
  void updateStatus();

private:
  void setupUI();
  void setupROS2();
  
  // ROS2 node
  rclcpp::Node::SharedPtr node_;
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr light1_control_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr light2_control_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr light1_brightness_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr light2_brightness_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trigger_source_pub_;
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr light1_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr light2_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr light1_brightness_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr light2_brightness_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_source_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connection_status_sub_;
  
  // Services
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr light1_service_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr light2_service_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr get_status_service_client_;
  
  // Timer for ROS2 spinning
  QTimer* ros_timer_;
  QTimer* status_timer_;
  
  // UI Components
  QWidget* central_widget_;
  
  // Connection status
  QLabel* connection_status_label_;
  
  // Light 1 controls
  QGroupBox* light1_group_;
  QPushButton* light1_toggle_btn_;
  QLabel* light1_status_label_;
  QSlider* light1_brightness_slider_;
  QLabel* light1_brightness_label_;
  bool light1_enabled_;
  int light1_brightness_;
  
  // Light 2 controls
  QGroupBox* light2_group_;
  QPushButton* light2_toggle_btn_;
  QLabel* light2_status_label_;
  QSlider* light2_brightness_slider_;
  QLabel* light2_brightness_label_;
  bool light2_enabled_;
  int light2_brightness_;
  
  // Trigger source
  QGroupBox* trigger_group_;
  QComboBox* trigger_source_combo_;
  QLabel* trigger_status_label_;
  std::string current_trigger_source_;
  
  // Status refresh button
  QPushButton* refresh_status_btn_;
  
  // Callbacks
  void light1StatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void light2StatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void light1BrightnessCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void light2BrightnessCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void triggerSourceCallback(const std_msgs::msg::String::SharedPtr msg);
  void connectionStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
  
  void updateUI();
};

#endif // LIGHT_CONTROLLER_GUI_HPP

