#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rs485_interface/motor/zd_motor/zd_motor_new.hpp"
#include "rs485_interface/common/rs485_client.hpp"

#include <memory>
#include <string>

class TransportLogicNode : public rclcpp::Node
{
public:
  TransportLogicNode()
  : Node("transport_logic_node")
  {
    // 声明参数
    this->declare_parameter<std::string>("motor_device_path", "/dev/ttyUSB1");
    this->declare_parameter<int>("motor_baud_rate", 19200);
    this->declare_parameter<int>("motor_slave_address", 1);
    this->declare_parameter<int>("reverse_speed_rpm", 1000);
    this->declare_parameter<int>("forward_speed_rpm", 1000);
    this->declare_parameter<std::string>("microswitch_topic", "/microswitch/data");

    std::string motor_device_path = this->get_parameter("motor_device_path").as_string();
    int motor_baud_rate = this->get_parameter("motor_baud_rate").as_int();
    int motor_slave_address = this->get_parameter("motor_slave_address").as_int();
    reverse_speed_rpm_ = this->get_parameter("reverse_speed_rpm").as_int();
    forward_speed_rpm_ = this->get_parameter("forward_speed_rpm").as_int();
    std::string microswitch_topic = this->get_parameter("microswitch_topic").as_string();

    // 创建 RS485 客户端
    rs485_interface::RS485Client::BaudRate baud;
    switch (motor_baud_rate) {
      case 2400: baud = rs485_interface::RS485Client::BaudRate::BAUD_2400; break;
      case 4800: baud = rs485_interface::RS485Client::BaudRate::BAUD_4800; break;
      case 9600: baud = rs485_interface::RS485Client::BaudRate::BAUD_9600; break;
      case 19200: baud = rs485_interface::RS485Client::BaudRate::BAUD_19200; break;
      case 38400: baud = rs485_interface::RS485Client::BaudRate::BAUD_38400; break;
      case 57600: baud = rs485_interface::RS485Client::BaudRate::BAUD_57600; break;
      case 115200: baud = rs485_interface::RS485Client::BaudRate::BAUD_115200; break;
      case 256000: baud = rs485_interface::RS485Client::BaudRate::BAUD_256000; break;
      default: baud = rs485_interface::RS485Client::BaudRate::BAUD_19200; break;
    }

    rs485_client_ = std::make_shared<rs485_interface::RS485Client>(
      motor_device_path,
      baud,
      rs485_interface::RS485Client::Parity::NONE,
      1000
    );

    if (!rs485_client_->open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open RS485 device: %s - %s",
                   motor_device_path.c_str(), rs485_client_->getLastError().c_str());
      throw std::runtime_error("Failed to open RS485 device");
    }

    RCLCPP_INFO(this->get_logger(), "RS485 device opened: %s", motor_device_path.c_str());

    // 创建 ZD motor
    motor_ = std::make_shared<rs485_interface::ZdMotor>(rs485_client_, motor_slave_address);

    if (!motor_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize ZD motor: %s",
                   motor_->getLastError().c_str());
      throw std::runtime_error("Failed to initialize ZD motor");
    }

    RCLCPP_INFO(this->get_logger(), "ZD motor initialized (slave address: %d)", motor_slave_address);

    // 订阅 microswitch 话题
    microswitch_sub_ = this->create_subscription<std_msgs::msg::String>(
      microswitch_topic,
      10,
      std::bind(&TransportLogicNode::microswitchCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Subscribed to microswitch topic: %s", microswitch_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Reverse speed: %d RPM, Forward speed: %d RPM",
                reverse_speed_rpm_, forward_speed_rpm_);

    current_state_ = "";
  }

  ~TransportLogicNode()
  {
    if (motor_) {
      motor_->setControlCommand(rs485_interface::ZdMotor::ControlCommand::STOP);
    }
    if (rs485_client_) {
      rs485_client_->close();
    }
  }

private:
  void microswitchCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string state = msg->data;
    
    // 去除可能的换行符和空格
    while (!state.empty() && (state.back() == '\n' || state.back() == '\r' || state.back() == ' ')) {
      state.pop_back();
    }
    
    // 去除前导空格
    while (!state.empty() && state.front() == ' ') {
      state.erase(0, 1);
    }

    // 只处理 vertical 相关的状态，忽略其他（如 horizontal）
    if (state.find("vertical") == std::string::npos) {
      // 忽略 non-vertical 状态，但不更新 current_state_
      return;
    }

    // 只接受 vertical0 和 vertical1，忽略其他包含 vertical 的字符串（如 verticalvertical0）
    if (state != "vertical0" && state != "vertical1") {
      RCLCPP_DEBUG(this->get_logger(), "Ignoring invalid vertical state: '%s'", state.c_str());
      return;
    }

    // 如果状态没有变化，直接返回
    if (state == current_state_) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Microswitch state changed: '%s' -> '%s'",
                current_state_.c_str(), state.c_str());

    std::string previous_state = current_state_;
    current_state_ = state;

    // 处理状态变化
    if (state == "vertical0") {
      // 如果从 vertical1 变回 vertical0，立即停止
      if (previous_state == "vertical1") {
        RCLCPP_INFO(this->get_logger(), "Stopping motor (transitioned to vertical0)");
        if (!motor_->setControlCommand(rs485_interface::ZdMotor::ControlCommand::STOP)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to stop motor: %s", motor_->getLastError().c_str());
        }
      } else {
        // 持续反转
        RCLCPP_INFO(this->get_logger(), "Setting motor to reverse at %d RPM", reverse_speed_rpm_);
        if (!motor_->setSpeedRPM(reverse_speed_rpm_)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to set reverse speed: %s", motor_->getLastError().c_str());
        } else if (!motor_->setControlCommand(rs485_interface::ZdMotor::ControlCommand::REVERSE)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to set reverse command: %s", motor_->getLastError().c_str());
        }
      }
    } else if (state == "vertical1") {
      // 立即正转
      RCLCPP_INFO(this->get_logger(), "Setting motor to forward at %d RPM", forward_speed_rpm_);
      if (!motor_->setSpeedRPM(forward_speed_rpm_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set forward speed: %s", motor_->getLastError().c_str());
      } else if (!motor_->setControlCommand(rs485_interface::ZdMotor::ControlCommand::FORWARD)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set forward command: %s", motor_->getLastError().c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown microswitch state: '%s'", state.c_str());
    }
  }

  std::shared_ptr<rs485_interface::RS485Client> rs485_client_;
  std::shared_ptr<rs485_interface::ZdMotor> motor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr microswitch_sub_;
  
  int reverse_speed_rpm_;
  int forward_speed_rpm_;
  std::string current_state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TransportLogicNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("transport_logic_node"), "Exception: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
