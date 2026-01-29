#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rs485_interface/motor/zd_motor/zd_motor_new.hpp"
#include "rs485_interface/motor/lc_servo_motor/lc_servo_motor_new.hpp"
#include "rs485_interface/common/rs485_client.hpp"

#include <memory>
#include <string>
#include <mutex>

class TransportLogicNode : public rclcpp::Node
{
public:
  TransportLogicNode()
  : Node("transport_logic_node")
  {
    this->declare_parameter<bool>("enable_zd_motor", true);
    this->declare_parameter<bool>("enable_servo_motor", true);
    this->declare_parameter<std::string>("motor_device_path", "/dev/ttyACM1");
    this->declare_parameter<int>("motor_baud_rate", 19200);
    this->declare_parameter<int>("motor_slave_address", 1);
    this->declare_parameter<int>("reverse_speed_rpm", 1000);
    this->declare_parameter<int>("forward_speed_rpm", 1000);
    this->declare_parameter<std::string>("servo_device_path", "/dev/ttyACM1");
    this->declare_parameter<int>("servo_baud_rate", 19200);
    this->declare_parameter<int>("servo_slave_address", 2);
    this->declare_parameter<double>("servo_forward_speed_rpm", 1000.0);
    this->declare_parameter<double>("servo_reverse_speed_rpm", 1000.0);
    this->declare_parameter<std::string>("microswitch_topic", "/microswitch/data");
    enable_zd_motor_ = this->get_parameter("enable_zd_motor").as_bool();
    enable_servo_motor_ = this->get_parameter("enable_servo_motor").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "ZD Motor enabled: %s, LC Servo Motor enabled: %s",
                enable_zd_motor_ ? "true" : "false",
                enable_servo_motor_ ? "true" : "false");

    std::string microswitch_topic = this->get_parameter("microswitch_topic").as_string();

    if (enable_zd_motor_) {
      std::string motor_device_path = this->get_parameter("motor_device_path").as_string();
      int motor_baud_rate = this->get_parameter("motor_baud_rate").as_int();
      int motor_slave_address = this->get_parameter("motor_slave_address").as_int();
      reverse_speed_rpm_ = this->get_parameter("reverse_speed_rpm").as_int();
      forward_speed_rpm_ = this->get_parameter("forward_speed_rpm").as_int();

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

      std::string servo_path = enable_servo_motor_ ? this->get_parameter("servo_device_path").as_string() : "";
      bool same_device = enable_servo_motor_ && (motor_device_path == servo_path);
      
      if (same_device) {
        RCLCPP_INFO(this->get_logger(), "ZD Motor and LC Servo Motor share the same device: %s", motor_device_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Will create shared RS485 client when initializing LC Servo Motor");
      } else {
        rs485_client_ = std::make_shared<rs485_interface::RS485Client>(
          motor_device_path,
          baud,
          rs485_interface::RS485Client::Parity::NONE,
          1000
        );

        if (!rs485_client_->open()) {
          RCLCPP_ERROR(this->get_logger(), "Failed to open ZD Motor RS485 device: %s - %s",
                       motor_device_path.c_str(), rs485_client_->getLastError().c_str());
          throw std::runtime_error("Failed to open ZD Motor RS485 device");
        }

        RCLCPP_INFO(this->get_logger(), "ZD Motor RS485 device opened: %s", motor_device_path.c_str());
        
        motor_ = std::make_shared<rs485_interface::ZdMotor>(rs485_client_, motor_slave_address);

        if (!motor_->initialize()) {
          RCLCPP_WARN(this->get_logger(), "Failed to initialize ZD motor: %s (will continue anyway)",
                       motor_->getLastError().c_str());
          RCLCPP_WARN(this->get_logger(), "Motor may still work, but some initialization steps failed");
        } else {
          RCLCPP_INFO(this->get_logger(), "ZD motor initialized (slave address: %d)", motor_slave_address);
        }
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "ZD Motor is disabled");
    }

    if (enable_servo_motor_) {
      std::string servo_device_path = this->get_parameter("servo_device_path").as_string();
      int servo_baud_rate = this->get_parameter("servo_baud_rate").as_int();
      int servo_slave_address = this->get_parameter("servo_slave_address").as_int();
      servo_forward_speed_rpm_ = this->get_parameter("servo_forward_speed_rpm").as_double();
      servo_reverse_speed_rpm_ = this->get_parameter("servo_reverse_speed_rpm").as_double();

      rs485_interface::RS485Client::BaudRate servo_baud;
      switch (servo_baud_rate) {
        case 2400: servo_baud = rs485_interface::RS485Client::BaudRate::BAUD_2400; break;
        case 4800: servo_baud = rs485_interface::RS485Client::BaudRate::BAUD_4800; break;
        case 9600: servo_baud = rs485_interface::RS485Client::BaudRate::BAUD_9600; break;
        case 19200: servo_baud = rs485_interface::RS485Client::BaudRate::BAUD_19200; break;
        case 38400: servo_baud = rs485_interface::RS485Client::BaudRate::BAUD_38400; break;
        case 57600: servo_baud = rs485_interface::RS485Client::BaudRate::BAUD_57600; break;
        case 115200: servo_baud = rs485_interface::RS485Client::BaudRate::BAUD_115200; break;
        case 256000: servo_baud = rs485_interface::RS485Client::BaudRate::BAUD_256000; break;
        default: servo_baud = rs485_interface::RS485Client::BaudRate::BAUD_19200; break;
      }

      std::string motor_path = enable_zd_motor_ ? this->get_parameter("motor_device_path").as_string() : "";
      bool same_device = enable_zd_motor_ && (servo_device_path == motor_path);
      
      if (same_device && rs485_client_) {
        servo_rs485_client_ = rs485_client_;
        RCLCPP_INFO(this->get_logger(), "Sharing RS485 client with ZD Motor (both use NONE parity)");
      } else {
        servo_rs485_client_ = std::make_shared<rs485_interface::RS485Client>(
          servo_device_path,
          servo_baud,
          rs485_interface::RS485Client::Parity::NONE,
          1000
        );

        if (!servo_rs485_client_->open()) {
          RCLCPP_ERROR(this->get_logger(), "Failed to open LC Servo RS485 device: %s - %s",
                       servo_device_path.c_str(), servo_rs485_client_->getLastError().c_str());
          throw std::runtime_error("Failed to open LC Servo RS485 device");
        }

        RCLCPP_INFO(this->get_logger(), "LC Servo RS485 device opened: %s", servo_device_path.c_str());
        
        if (same_device && enable_zd_motor_ && !rs485_client_) {
          rs485_client_ = servo_rs485_client_;
          RCLCPP_INFO(this->get_logger(), "ZD Motor will use LC Servo Motor's RS485 client (both use NONE parity)");
          
          int motor_slave_address = this->get_parameter("motor_slave_address").as_int();
          motor_ = std::make_shared<rs485_interface::ZdMotor>(rs485_client_, motor_slave_address);
          
          if (!motor_->initialize()) {
            RCLCPP_WARN(this->get_logger(), "Failed to initialize ZD motor: %s (will continue anyway)",
                         motor_->getLastError().c_str());
            RCLCPP_WARN(this->get_logger(), "Motor may still work, but some initialization steps failed");
          } else {
            RCLCPP_INFO(this->get_logger(), "ZD motor initialized (slave address: %d)", motor_slave_address);
          }
        }
      }

      servo_motor_ = std::make_shared<rs485_interface::LcServoMotor>(servo_rs485_client_, servo_slave_address);

      if (!servo_motor_->initializeSpeedControl()) {
        RCLCPP_WARN(this->get_logger(), "Failed to initialize LC Servo motor: %s (will continue anyway)",
                     servo_motor_->getLastError().c_str());
        RCLCPP_WARN(this->get_logger(), "Motor may still work, but some initialization steps failed");
      } else {
        RCLCPP_INFO(this->get_logger(), "LC Servo motor initialized (slave address: %d)", servo_slave_address);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "LC Servo Motor is disabled");
    }

    microswitch_sub_ = this->create_subscription<std_msgs::msg::String>(
      microswitch_topic,
      10,
      std::bind(&TransportLogicNode::microswitchCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Subscribed to microswitch topic: %s", microswitch_topic.c_str());
    if (enable_zd_motor_) {
      RCLCPP_INFO(this->get_logger(), "ZD Motor - Reverse speed: %d RPM, Forward speed: %d RPM",
                  reverse_speed_rpm_, forward_speed_rpm_);
    }
    if (enable_servo_motor_) {
      RCLCPP_INFO(this->get_logger(), "LC Servo - Forward speed: %.1f RPM, Reverse speed: %.1f RPM",
                  servo_forward_speed_rpm_, servo_reverse_speed_rpm_);
    }

    current_vertical_state_ = "";
    current_horizontal_state_ = "";
    pending_vertical_state_ = "";
    pending_horizontal_state_ = "";
    vertical_state_count_ = 0;
    horizontal_state_count_ = 0;
    debounce_threshold_ = 3;
  }

  ~TransportLogicNode()
  {
    if (motor_) {
      motor_->setControlCommand(rs485_interface::ZdMotor::ControlCommand::STOP);
    }
    if (servo_motor_) {
      servo_motor_->setDirection(rs485_interface::LcServoMotor::Direction::STOP);
      servo_motor_->setEnable(rs485_interface::LcServoMotor::EnableState::DISABLE);
    }
    if (rs485_client_) {
      rs485_client_->close();
    }
    if (servo_rs485_client_) {
      servo_rs485_client_->close();
    }
  }

private:
  void microswitchCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string state = msg->data;
    
    while (!state.empty() && (state.back() == '\n' || state.back() == '\r' || state.back() == ' ')) {
      state.pop_back();
    }
    
    while (!state.empty() && state.front() == ' ') {
      state.erase(0, 1);
    }

    if (enable_zd_motor_ && state.find("vertical_max") != std::string::npos) {
      if (state == "vertical_max_0" || state == "vertical_max_1") {
        if (state == pending_vertical_state_) {
          vertical_state_count_++;
          if (vertical_state_count_ >= debounce_threshold_) {
            handleVerticalState(state);
            vertical_state_count_ = 0;
            pending_vertical_state_ = "";
          }
        } else {
          pending_vertical_state_ = state;
          vertical_state_count_ = 1;
        }
      }
    }
    
    if (enable_servo_motor_ && state.find("horizontal_max") != std::string::npos) {
      if (state == "horizontal_max_0" || state == "horizontal_max_1") {
        if (state == pending_horizontal_state_) {
          horizontal_state_count_++;
          if (horizontal_state_count_ >= debounce_threshold_) {
            handleHorizontalState(state);
            horizontal_state_count_ = 0;
            pending_horizontal_state_ = "";
          }
        } else {
          pending_horizontal_state_ = state;
          horizontal_state_count_ = 1;
        }
      }
    }
  }

  void handleVerticalState(const std::string & state)
  {
    if (state != "vertical_max_0" && state != "vertical_max_1") {
      RCLCPP_DEBUG(this->get_logger(), "Ignoring invalid vertical state: '%s'", state.c_str());
      return;
    }

    if (state == current_vertical_state_) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Vertical state changed: '%s' -> '%s'",
                current_vertical_state_.c_str(), state.c_str());

    std::string previous_state = current_vertical_state_;
    current_vertical_state_ = state;

    if (state == "vertical_max_0") {
      if (previous_state == "vertical_max_1") {
        RCLCPP_INFO(this->get_logger(), "ZD motor left microswitch, stopping (homing complete)");
        if (!motor_->setControlCommand(rs485_interface::ZdMotor::ControlCommand::STOP)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to stop ZD motor: %s", motor_->getLastError().c_str());
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "ZD motor reversing to find home (vertical_max_0) at %d RPM", reverse_speed_rpm_);
        if (!motor_->setSpeedRPM(reverse_speed_rpm_)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to set reverse speed: %s", motor_->getLastError().c_str());
        } else if (!motor_->setControlCommand(rs485_interface::ZdMotor::ControlCommand::REVERSE)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to set reverse command: %s", motor_->getLastError().c_str());
        }
      }
    } else if (state == "vertical_max_1") {
      RCLCPP_INFO(this->get_logger(), "ZD motor hit microswitch, forwarding to leave (vertical_max_1) at %d RPM", forward_speed_rpm_);
      if (!motor_->setSpeedRPM(forward_speed_rpm_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set forward speed: %s", motor_->getLastError().c_str());
      } else if (!motor_->setControlCommand(rs485_interface::ZdMotor::ControlCommand::FORWARD)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set forward command: %s", motor_->getLastError().c_str());
      }
    }
  }

  void handleHorizontalState(const std::string & state)
  {
    if (state != "horizontal_max_0" && state != "horizontal_max_1") {
      RCLCPP_DEBUG(this->get_logger(), "Ignoring invalid horizontal state: '%s'", state.c_str());
      return;
    }

    if (state == current_horizontal_state_) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Horizontal state changed: '%s' -> '%s'",
                current_horizontal_state_.c_str(), state.c_str());

    std::string previous_state = current_horizontal_state_;
    current_horizontal_state_ = state;

    if (state == "horizontal_max_0") {
      if (previous_state == "horizontal_max_1") {
        RCLCPP_INFO(this->get_logger(), "LC Servo motor left microswitch, stopping (homing complete)");
        servo_motor_->setEnable(rs485_interface::LcServoMotor::EnableState::ENABLE);
        if (!servo_motor_->setDirection(rs485_interface::LcServoMotor::Direction::STOP)) {
          RCLCPP_WARN(this->get_logger(), "Failed to stop LC Servo motor: %s (continuing anyway)", servo_motor_->getLastError().c_str());
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "LC Servo motor forwarding to find home (horizontal_max_0) at %.1f RPM", servo_forward_speed_rpm_);
        servo_motor_->setEnable(rs485_interface::LcServoMotor::EnableState::ENABLE);
        if (!servo_motor_->setSpeedRPM(servo_forward_speed_rpm_)) {
          RCLCPP_WARN(this->get_logger(), "Failed to set forward speed: %s (continuing anyway)", servo_motor_->getLastError().c_str());
        }
        if (!servo_motor_->setDirection(rs485_interface::LcServoMotor::Direction::FORWARD)) {
          RCLCPP_WARN(this->get_logger(), "Failed to set forward direction: %s (continuing anyway)", servo_motor_->getLastError().c_str());
        }
      }
    } else if (state == "horizontal_max_1") {
      RCLCPP_INFO(this->get_logger(), "LC Servo motor hit microswitch, reversing to leave (horizontal_max_1) at %.1f RPM", servo_reverse_speed_rpm_);
      servo_motor_->setEnable(rs485_interface::LcServoMotor::EnableState::ENABLE);
      if (!servo_motor_->setSpeedRPM(servo_reverse_speed_rpm_)) {
        RCLCPP_WARN(this->get_logger(), "Failed to set reverse speed: %s (continuing anyway)", servo_motor_->getLastError().c_str());
      }
      if (!servo_motor_->setDirection(rs485_interface::LcServoMotor::Direction::REVERSE)) {
        RCLCPP_WARN(this->get_logger(), "Failed to set reverse direction: %s (continuing anyway)", servo_motor_->getLastError().c_str());
      }
    }
  }

  bool enable_zd_motor_;
  bool enable_servo_motor_;
  std::shared_ptr<rs485_interface::RS485Client> rs485_client_;
  std::shared_ptr<rs485_interface::ZdMotor> motor_;
  std::shared_ptr<rs485_interface::RS485Client> servo_rs485_client_;
  std::shared_ptr<rs485_interface::LcServoMotor> servo_motor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr microswitch_sub_;
  
  int reverse_speed_rpm_;
  int forward_speed_rpm_;
  double servo_forward_speed_rpm_;
  double servo_reverse_speed_rpm_;
  std::string current_vertical_state_;
  std::string current_horizontal_state_;
  std::string pending_vertical_state_;
  std::string pending_horizontal_state_;
  int vertical_state_count_;
  int horizontal_state_count_;
  int debounce_threshold_;
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
