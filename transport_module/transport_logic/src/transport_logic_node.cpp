#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rs485_interface/motor/zd_motor/zd_motor_new.hpp"
#include "rs485_interface/motor/lc_servo_motor/lc_servo_motor_new.hpp"
#include "rs485_interface/common/rs485_client.hpp"

#include <memory>
#include <string>
#include <mutex>
#include <chrono>
#include <thread>

class TransportLogicNode : public rclcpp::Node
{
public:
  TransportLogicNode()
  : Node("transport_logic_node")
  {
    this->declare_parameter<bool>("enable_zd_motor", true);
    this->declare_parameter<bool>("enable_servo_motor", true);
    this->declare_parameter<int>("work_speed_rpm", 500);
    this->declare_parameter<int>("work_lift_speed_rpm", 300);
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
    work_speed_rpm_ = this->get_parameter("work_speed_rpm").as_int();
    work_lift_speed_rpm_ = this->get_parameter("work_lift_speed_rpm").as_int();
    
    RCLCPP_INFO(this->get_logger(), "ZD Motor enabled: %s, LC Servo Motor enabled: %s",
                enable_zd_motor_ ? "true" : "false",
                enable_servo_motor_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "WORK speed (ZD motor id=3): %d RPM (REVERSE)", work_speed_rpm_);
    RCLCPP_INFO(this->get_logger(), "WORK lift speed (ZD motor primary): %d RPM (FORWARD after 10s)", work_lift_speed_rpm_);

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

    // Extra ZD motor for WORK (slave id = 3). Reuses the same RS485 client/settings.
    if (enable_zd_motor_ && rs485_client_) {
      motor_work_ = std::make_shared<rs485_interface::ZdMotor>(rs485_client_, 3);
      if (!motor_work_->initialize()) {
        RCLCPP_WARN(this->get_logger(), "Failed to initialize WORK ZD motor (slave address: 3): %s (will continue anyway)",
                    motor_work_->getLastError().c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "WORK ZD motor initialized (slave address: 3)");
      }
    } else if (enable_zd_motor_ && !rs485_client_) {
      RCLCPP_WARN(this->get_logger(), "WORK ZD motor (slave address: 3) not created because RS485 client is not available");
    }

    microswitch_sub_ = this->create_subscription<std_msgs::msg::String>(
      microswitch_topic,
      10,
      std::bind(&TransportLogicNode::microswitchCallback, this, std::placeholders::_1)
    );

    // Publishers for status
    state_machine_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "transport_logic/state_machine_state", 10);
    microswitch_status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "transport_logic/microswitch_status", 10);
    vertical_max_pub_ = this->create_publisher<std_msgs::msg::String>(
      "transport_logic/vertical_max", 10);
    vertical_min_pub_ = this->create_publisher<std_msgs::msg::String>(
      "transport_logic/vertical_min", 10);
    horizontal_max_pub_ = this->create_publisher<std_msgs::msg::String>(
      "transport_logic/horizontal_max", 10);
    horizontal_min_pub_ = this->create_publisher<std_msgs::msg::String>(
      "transport_logic/horizontal_min", 10);
    zd_motor_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "transport_logic/zd_motor_state", 10);
    servo_motor_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "transport_logic/servo_motor_state", 10);
    motor_work_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "transport_logic/motor_work_state", 10);

    // Subscribers for manual control
    manual_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "transport_logic/manual_mode",
      10,
      std::bind(&TransportLogicNode::manualModeCallback, this, std::placeholders::_1)
    );
    state_machine_set_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "transport_logic/set_state",
      10,
      std::bind(&TransportLogicNode::setStateCallback, this, std::placeholders::_1)
    );
    confirm_wait_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "transport_logic/confirm_wait",
      10,
      std::bind(&TransportLogicNode::confirmWaitCallback, this, std::placeholders::_1)
    );
    zd_motor_manual_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "transport_logic/zd_motor/manual_speed",
      10,
      std::bind(&TransportLogicNode::zdMotorManualSpeedCallback, this, std::placeholders::_1)
    );
    zd_motor_manual_direction_sub_ = this->create_subscription<std_msgs::msg::String>(
      "transport_logic/zd_motor/manual_direction",
      10,
      std::bind(&TransportLogicNode::zdMotorManualDirectionCallback, this, std::placeholders::_1)
    );
    servo_motor_manual_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "transport_logic/servo_motor/manual_speed",
      10,
      std::bind(&TransportLogicNode::servoMotorManualSpeedCallback, this, std::placeholders::_1)
    );
    servo_motor_manual_direction_sub_ = this->create_subscription<std_msgs::msg::String>(
      "transport_logic/servo_motor/manual_direction",
      10,
      std::bind(&TransportLogicNode::servoMotorManualDirectionCallback, this, std::placeholders::_1)
    );
    servo_motor_manual_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "transport_logic/servo_motor/manual_enable",
      10,
      std::bind(&TransportLogicNode::servoMotorManualEnableCallback, this, std::placeholders::_1)
    );

    // Status publishing timer (10 Hz)
    using namespace std::chrono_literals;
    status_timer_ = this->create_wall_timer(
      100ms,
      std::bind(&TransportLogicNode::publishStatus, this)
    );

    // Initialize manual mode and motor state tracking (default to manual mode)
    manual_mode_ = true;
    zd_motor_current_speed_ = 0;
    zd_motor_current_direction_ = "STOP";
    servo_motor_current_speed_ = 0.0;
    servo_motor_current_direction_ = "STOP";
    servo_motor_enabled_ = false;
    motor_work_current_speed_ = 0;
    motor_work_current_direction_ = "STOP";
    last_microswitch_status_ = "";

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

    // Main state machine framework (start from INIT by default)
    main_state_ = MainState::INIT;
    last_logged_state_ = main_state_;
    logStateEnter(main_state_, "startup");
    init_vertical_done_ = !enable_zd_motor_;
    init_horizontal_done_ = !enable_servo_motor_;

    using namespace std::chrono_literals;
    state_timer_ = this->create_wall_timer(
      100ms,
      std::bind(&TransportLogicNode::tick, this)
    );
  }

  ~TransportLogicNode()
  {
    if (motor_) {
      setZdMotorCommandRetry(motor_, rs485_interface::ZdMotor::ControlCommand::STOP);
    }
    if (motor_work_) {
      setZdMotorCommandRetry(motor_work_, rs485_interface::ZdMotor::ControlCommand::STOP);
    }
    if (servo_motor_) {
      setServoDirectionRetry(servo_motor_, rs485_interface::LcServoMotor::Direction::STOP);
      setServoEnableRetry(servo_motor_, rs485_interface::LcServoMotor::EnableState::DISABLE);
    }
    if (rs485_client_) {
      rs485_client_->close();
    }
    if (servo_rs485_client_) {
      servo_rs485_client_->close();
    }
  }

private:
  enum class MainState
  {
    INIT,
    WAIT,
    PRE,
    WORK,
    FINISH
  };

  static const char * toString(MainState s)
  {
    switch (s) {
      case MainState::INIT: return "INIT";
      case MainState::WAIT: return "WAIT";
      case MainState::PRE: return "PRE";
      case MainState::WORK: return "WORK";
      case MainState::FINISH: return "FINISH";
      default: return "UNKNOWN";
    }
  }

  void logStateEnter(MainState s, const std::string & reason)
  {
    RCLCPP_INFO(this->get_logger(), "MainState enter: %s (reason: %s)", toString(s), reason.c_str());
  }

  void setMainState(MainState next, const std::string & reason)
  {
    if (next == main_state_) {
      return;
    }
    main_state_ = next;
    logStateEnter(main_state_, reason);

    // On-enter resets / defaults
    if (main_state_ == MainState::INIT) {
      init_vertical_done_ = !enable_zd_motor_;
      init_horizontal_done_ = !enable_servo_motor_;
    } else if (main_state_ == MainState::WAIT) {
      // WAIT state: waiting for user confirmation to proceed to PRE
    } else if (main_state_ == MainState::PRE) {
      pre_vertical_done_ = !enable_zd_motor_;
      pre_horizontal_done_ = !enable_servo_motor_;
      pre_vertical_reversing_back_ = false;
      pre_horizontal_reversing_back_ = false;
    } else if (main_state_ == MainState::WORK) {
      motor_work_started_ = false;
      work_vertical_min_state_.clear();
      work_pending_vertical_min_state_.clear();
      work_vertical_min_state_count_ = 0;
      work_waiting_for_vertical_min_ = false;
      work_forwarding_to_vertical_min_ = false;
      work_wait_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
      work_motor_forward_started_ = false;
      work_motor_work_waiting_stop_ = false;
      work_motor_work_stop_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    }
  }

  // Helper: send ZD motor speed command 3 times
  bool setZdMotorSpeedRetry(std::shared_ptr<rs485_interface::ZdMotor>& motor, int speed_rpm)
  {
    bool success = false;
    for (int i = 0; i < 3; ++i) {
      if (motor->setSpeedRPM(speed_rpm)) {
        success = true;
      }
      if (i < 2) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    if (!success) {
      RCLCPP_WARN(this->get_logger(), "Failed to set ZD motor speed to %d RPM after 3 retries", speed_rpm);
    } else {
      // Update state tracking
      if (motor == motor_) {
        zd_motor_current_speed_ = speed_rpm;
      } else if (motor == motor_work_) {
        motor_work_current_speed_ = speed_rpm;
      }
    }
    return success;
  }

  // Helper: send ZD motor control command 3 times
  bool setZdMotorCommandRetry(std::shared_ptr<rs485_interface::ZdMotor>& motor, rs485_interface::ZdMotor::ControlCommand cmd)
  {
    bool success = false;
    for (int i = 0; i < 3; ++i) {
      if (motor->setControlCommand(cmd)) {
        success = true;
      }
      if (i < 2) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    if (!success) {
      RCLCPP_WARN(this->get_logger(), "Failed to set ZD motor command after 3 retries");
    } else {
      // Update state tracking
      std::string dir_str;
      if (cmd == rs485_interface::ZdMotor::ControlCommand::FORWARD) {
        dir_str = "FORWARD";
      } else if (cmd == rs485_interface::ZdMotor::ControlCommand::REVERSE) {
        dir_str = "REVERSE";
      } else {
        dir_str = "STOP";
      }
      if (motor == motor_) {
        zd_motor_current_direction_ = dir_str;
      } else if (motor == motor_work_) {
        motor_work_current_direction_ = dir_str;
      }
    }
    return success;
  }

  // Helper: send servo motor speed command 3 times
  bool setServoSpeedRetry(std::shared_ptr<rs485_interface::LcServoMotor>& servo, double speed_rpm)
  {
    bool success = false;
    for (int i = 0; i < 3; ++i) {
      if (servo->setSpeedRPM(speed_rpm)) {
        success = true;
      }
      if (i < 2) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    if (!success) {
      RCLCPP_WARN(this->get_logger(), "Failed to set servo speed to %.1f RPM after 3 retries", speed_rpm);
    } else {
      servo_motor_current_speed_ = speed_rpm;
    }
    return success;
  }

  // Helper: send servo motor direction command 3 times
  bool setServoDirectionRetry(std::shared_ptr<rs485_interface::LcServoMotor>& servo, rs485_interface::LcServoMotor::Direction dir)
  {
    bool success = false;
    for (int i = 0; i < 3; ++i) {
      if (servo->setDirection(dir)) {
        success = true;
      }
      if (i < 2) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    if (!success) {
      RCLCPP_WARN(this->get_logger(), "Failed to set servo direction after 3 retries");
    } else {
      if (dir == rs485_interface::LcServoMotor::Direction::FORWARD) {
        servo_motor_current_direction_ = "FORWARD";
      } else if (dir == rs485_interface::LcServoMotor::Direction::REVERSE) {
        servo_motor_current_direction_ = "REVERSE";
      } else {
        servo_motor_current_direction_ = "STOP";
      }
    }
    return success;
  }

  // Helper: send servo enable command 3 times
  void setServoEnableRetry(std::shared_ptr<rs485_interface::LcServoMotor>& servo, rs485_interface::LcServoMotor::EnableState enable)
  {
    for (int i = 0; i < 3; ++i) {
      servo->setEnable(enable);
      if (i < 2) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    servo_motor_enabled_ = (enable == rs485_interface::LcServoMotor::EnableState::ENABLE);
  }

  void tick()
  {
    // Only log once per state change to avoid log spam
    if (last_logged_state_ != main_state_) {
      last_logged_state_ = main_state_;
    }

    // Skip state machine logic if in manual mode
    if (manual_mode_) {
      return;
    }

    switch (main_state_) {
      case MainState::INIT:
        tickInit();
        break;
      case MainState::WAIT:
        tickWait();
        break;
      case MainState::PRE:
        tickPre();
        break;
      case MainState::WORK:
        tickWork();
        break;
      case MainState::FINISH:
        tickFinish();
        break;
      default:
        break;
    }
  }

  void publishStatus()
  {
    // Publish state machine state
    auto state_msg = std_msgs::msg::String();
    state_msg.data = toString(main_state_);
    state_machine_state_pub_->publish(state_msg);

    // Publish microswitch status
    auto microswitch_msg = std_msgs::msg::String();
    microswitch_msg.data = last_microswitch_status_;
    microswitch_status_pub_->publish(microswitch_msg);

    // Publish individual microswitch states
    auto vertical_max_msg = std_msgs::msg::String();
    vertical_max_msg.data = current_vertical_state_.empty() ? "unknown" : current_vertical_state_;
    vertical_max_pub_->publish(vertical_max_msg);

    auto vertical_min_msg = std_msgs::msg::String();
    // Use work_vertical_min_state_ if in WORK state, otherwise use current_vertical_min_state_
    std::string vertical_min_state = (main_state_ == MainState::WORK && !work_vertical_min_state_.empty()) ?
                                     work_vertical_min_state_ : current_vertical_min_state_;
    vertical_min_msg.data = vertical_min_state.empty() ? "unknown" : vertical_min_state;
    vertical_min_pub_->publish(vertical_min_msg);

    auto horizontal_max_msg = std_msgs::msg::String();
    horizontal_max_msg.data = current_horizontal_state_.empty() ? "unknown" : current_horizontal_state_;
    horizontal_max_pub_->publish(horizontal_max_msg);

    auto horizontal_min_msg = std_msgs::msg::String();
    horizontal_min_msg.data = current_horizontal_min_state_.empty() ? "unknown" : current_horizontal_min_state_;
    horizontal_min_pub_->publish(horizontal_min_msg);

    // Publish ZD motor state (format: "speed,direction")
    auto zd_motor_msg = std_msgs::msg::String();
    zd_motor_msg.data = std::to_string(zd_motor_current_speed_) + "," + zd_motor_current_direction_;
    zd_motor_state_pub_->publish(zd_motor_msg);

    // Publish servo motor state (format: "speed,direction,enabled")
    auto servo_motor_msg = std_msgs::msg::String();
    servo_motor_msg.data = std::to_string(servo_motor_current_speed_) + "," + 
                          servo_motor_current_direction_ + "," + 
                          (servo_motor_enabled_ ? "true" : "false");
    servo_motor_state_pub_->publish(servo_motor_msg);

    // Publish motor_work state (format: "speed,direction")
    auto motor_work_msg = std_msgs::msg::String();
    motor_work_msg.data = std::to_string(motor_work_current_speed_) + "," + motor_work_current_direction_;
    motor_work_state_pub_->publish(motor_work_msg);
  }

  void manualModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    manual_mode_ = msg->data;
    if (manual_mode_) {
      RCLCPP_INFO(this->get_logger(), "Manual mode enabled - state machine disabled");
    } else {
      RCLCPP_INFO(this->get_logger(), "Manual mode disabled - state machine enabled");
    }
  }

  void setStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string state_str = msg->data;
    // Trim whitespace
    while (!state_str.empty() && (state_str.back() == '\n' || state_str.back() == '\r' || state_str.back() == ' ')) {
      state_str.pop_back();
    }
    while (!state_str.empty() && state_str.front() == ' ') {
      state_str.erase(0, 1);
    }

    MainState new_state;
    if (state_str == "INIT") {
      new_state = MainState::INIT;
    } else if (state_str == "WAIT") {
      new_state = MainState::WAIT;
    } else if (state_str == "PRE") {
      new_state = MainState::PRE;
    } else if (state_str == "WORK") {
      new_state = MainState::WORK;
    } else if (state_str == "FINISH") {
      new_state = MainState::FINISH;
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid state requested: %s", state_str.c_str());
      return;
    }

    setMainState(new_state, "manual_set");
    RCLCPP_INFO(this->get_logger(), "State manually set to: %s", state_str.c_str());
  }

  void confirmWaitCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && main_state_ == MainState::WAIT) {
      setMainState(MainState::PRE, "user_confirmed");
      RCLCPP_INFO(this->get_logger(), "WAIT state confirmed, transitioning to PRE");
    }
  }

  void zdMotorManualSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!manual_mode_ || !enable_zd_motor_ || !motor_) {
      return;
    }
    int speed = static_cast<int>(msg->data);
    zd_motor_current_speed_ = speed;
    setZdMotorSpeedRetry(motor_, speed);
    RCLCPP_INFO(this->get_logger(), "ZD Motor manual speed set to: %d RPM", speed);
  }

  void zdMotorManualDirectionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!manual_mode_ || !enable_zd_motor_ || !motor_) {
      return;
    }
    std::string dir_str = msg->data;
    // Trim whitespace
    while (!dir_str.empty() && (dir_str.back() == '\n' || dir_str.back() == '\r' || dir_str.back() == ' ')) {
      dir_str.pop_back();
    }
    while (!dir_str.empty() && dir_str.front() == ' ') {
      dir_str.erase(0, 1);
    }

    rs485_interface::ZdMotor::ControlCommand cmd;
    if (dir_str == "FORWARD") {
      cmd = rs485_interface::ZdMotor::ControlCommand::FORWARD;
      zd_motor_current_direction_ = "FORWARD";
    } else if (dir_str == "REVERSE") {
      cmd = rs485_interface::ZdMotor::ControlCommand::REVERSE;
      zd_motor_current_direction_ = "REVERSE";
    } else if (dir_str == "STOP") {
      cmd = rs485_interface::ZdMotor::ControlCommand::STOP;
      zd_motor_current_direction_ = "STOP";
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid ZD motor direction: %s", dir_str.c_str());
      return;
    }

    setZdMotorCommandRetry(motor_, cmd);
    RCLCPP_INFO(this->get_logger(), "ZD Motor manual direction set to: %s", dir_str.c_str());
  }

  void servoMotorManualSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!manual_mode_ || !enable_servo_motor_ || !servo_motor_) {
      return;
    }
    double speed = msg->data;
    servo_motor_current_speed_ = speed;
    setServoSpeedRetry(servo_motor_, speed);
    RCLCPP_INFO(this->get_logger(), "Servo Motor manual speed set to: %.1f RPM", speed);
  }

  void servoMotorManualDirectionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!manual_mode_ || !enable_servo_motor_ || !servo_motor_) {
      return;
    }
    std::string dir_str = msg->data;
    // Trim whitespace
    while (!dir_str.empty() && (dir_str.back() == '\n' || dir_str.back() == '\r' || dir_str.back() == ' ')) {
      dir_str.pop_back();
    }
    while (!dir_str.empty() && dir_str.front() == ' ') {
      dir_str.erase(0, 1);
    }

    rs485_interface::LcServoMotor::Direction dir;
    if (dir_str == "FORWARD") {
      dir = rs485_interface::LcServoMotor::Direction::FORWARD;
      servo_motor_current_direction_ = "FORWARD";
    } else if (dir_str == "REVERSE") {
      dir = rs485_interface::LcServoMotor::Direction::REVERSE;
      servo_motor_current_direction_ = "REVERSE";
    } else if (dir_str == "STOP") {
      dir = rs485_interface::LcServoMotor::Direction::STOP;
      servo_motor_current_direction_ = "STOP";
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid Servo motor direction: %s", dir_str.c_str());
      return;
    }

    setServoDirectionRetry(servo_motor_, dir);
    RCLCPP_INFO(this->get_logger(), "Servo Motor manual direction set to: %s", dir_str.c_str());
  }

  void servoMotorManualEnableCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!manual_mode_ || !enable_servo_motor_ || !servo_motor_) {
      return;
    }
    bool enable = msg->data;
    servo_motor_enabled_ = enable;
    rs485_interface::LcServoMotor::EnableState state = enable ? 
      rs485_interface::LcServoMotor::EnableState::ENABLE : 
      rs485_interface::LcServoMotor::EnableState::DISABLE;
    setServoEnableRetry(servo_motor_, state);
    RCLCPP_INFO(this->get_logger(), "Servo Motor manual enable set to: %s", enable ? "true" : "false");
  }

  // Framework placeholders: keep empty for now (no internal contents yet)
  void tickInit()
  {
    // Auto transition: INIT -> WAIT when homing is complete.
    if (init_vertical_done_ && init_horizontal_done_) {
      setMainState(MainState::WAIT, "init complete");
    }
  }
  void tickWait()
  {
    // WAIT state: do nothing, wait for user confirmation via topic
    // Transition to PRE is handled by confirmWaitCallback
  }
  void tickPre()
  {
    // Auto transition: PRE -> WORK when MIN positioning is complete.
    if (pre_vertical_done_ && pre_horizontal_done_) {
      setMainState(MainState::WORK, "pre complete");
    }
  }
  void tickWork()
  {
    // Only implement requested WORK behavior: run the extra ZD motor (slave id=3) in REVERSE.
    // Start motor_work_ when vertical_min is 1, continue for 2s after vertical_min becomes 0, then stop.
    if (motor_work_) {
      const auto now = this->get_clock()->now();
      
      // Check if 2s delay after vertical_min_0 is complete
      if (work_motor_work_waiting_stop_) {
        const double elapsed = (now - work_motor_work_stop_time_).seconds();
        if (elapsed >= 2.0) {
          RCLCPP_INFO(this->get_logger(), "WORK: 2s elapsed after vertical_min_0, stopping motor_work_");
          setZdMotorCommandRetry(motor_work_, rs485_interface::ZdMotor::ControlCommand::STOP);
          motor_work_started_ = false;
          work_motor_work_waiting_stop_ = false;
        }
      }
      
      // Start motor_work_ when vertical_min becomes 1
      if (work_vertical_min_state_ == "vertical_min_1" && !motor_work_started_ && !work_motor_work_waiting_stop_) {
        RCLCPP_INFO(this->get_logger(), "WORK: vertical_min is 1, starting motor_work_");
        setZdMotorSpeedRetry(motor_work_, work_speed_rpm_);
        setZdMotorCommandRetry(motor_work_, rs485_interface::ZdMotor::ControlCommand::REVERSE);
        motor_work_started_ = true;
        work_motor_work_waiting_stop_ = false;
      }
      
      // When vertical_min becomes 0, start 2s countdown
      if (work_vertical_min_state_ == "vertical_min_0" && motor_work_started_ && !work_motor_work_waiting_stop_) {
        RCLCPP_INFO(this->get_logger(), "WORK: vertical_min is 0, motor_work_ will stop after 2s");
        work_motor_work_waiting_stop_ = true;
        work_motor_work_stop_time_ = now;
      }
    }

    // WORK addition:
    // When vertical_min is 0, wait 10 seconds then forward ZD motor until vertical_min becomes 1.
    if (!enable_zd_motor_ || !motor_) {
      return;
    }

    const auto now = this->get_clock()->now();
    if (!work_waiting_for_vertical_min_ && !work_forwarding_to_vertical_min_ &&
        work_vertical_min_state_ == "vertical_min_0") {
      work_waiting_for_vertical_min_ = true;
      work_wait_start_time_ = now;
      RCLCPP_INFO(this->get_logger(), "WORK: detected vertical_min_0, waiting 10s before forwarding ZD motor");
    }

    if (work_waiting_for_vertical_min_) {
      const double elapsed = (now - work_wait_start_time_).seconds();
      if (elapsed >= 5.0) {
        work_waiting_for_vertical_min_ = false;
        work_forwarding_to_vertical_min_ = true;
        work_motor_forward_started_ = false;
        RCLCPP_INFO(this->get_logger(), "WORK: 10s elapsed, start forwarding ZD motor until vertical_min_1");
      }
    }

    if (work_forwarding_to_vertical_min_) {
      if (work_vertical_min_state_ == "vertical_min_1") {
        work_forwarding_to_vertical_min_ = false;
        work_motor_forward_started_ = false;
        RCLCPP_INFO(this->get_logger(), "WORK: reached vertical_min_1, stopping ZD motor");
        setZdMotorCommandRetry(motor_, rs485_interface::ZdMotor::ControlCommand::STOP);
      } else {
        // Keep forwarding; send commands once to avoid spamming.
        if (!work_motor_forward_started_) {
          setZdMotorSpeedRetry(motor_, work_lift_speed_rpm_);
          setZdMotorCommandRetry(motor_, rs485_interface::ZdMotor::ControlCommand::FORWARD);
          work_motor_forward_started_ = true;
        }
      }
    }
  }
  void tickFinish() {}

  void microswitchCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string state = msg->data;
    
    // Update last microswitch status for publishing
    last_microswitch_status_ = state;

    // Only process microswitch-driven motor logic in INIT / PRE / WORK states.
    // WAIT state doesn't process microswitch (waiting for user confirmation)
    if (main_state_ != MainState::INIT && main_state_ != MainState::PRE && main_state_ != MainState::WORK) {
      return;
    }

    // Skip state machine logic if in manual mode
    if (manual_mode_) {
      return;
    }
    
    while (!state.empty() && (state.back() == '\n' || state.back() == '\r' || state.back() == ' ')) {
      state.pop_back();
    }
    
    while (!state.empty() && state.front() == ' ') {
      state.erase(0, 1);
    }

    // ===================== INIT (existing behavior) =====================
    if (main_state_ == MainState::INIT && enable_zd_motor_ && state.find("vertical_max") != std::string::npos) {
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
    
    if (main_state_ == MainState::INIT && enable_servo_motor_ && state.find("horizontal_max") != std::string::npos) {
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

    // ===================== PRE (go to MIN and stop) =====================
    if (main_state_ == MainState::PRE && enable_zd_motor_ && state.find("vertical_min") != std::string::npos) {
      if (state == "vertical_min_0" || state == "vertical_min_1") {
        if (state == pending_vertical_min_state_) {
          vertical_min_state_count_++;
          if (vertical_min_state_count_ >= debounce_threshold_) {
            handleVerticalMinState(state);
            vertical_min_state_count_ = 0;
            pending_vertical_min_state_ = "";
          }
        } else {
          pending_vertical_min_state_ = state;
          vertical_min_state_count_ = 1;
        }
      }
    }

    if (main_state_ == MainState::PRE && enable_servo_motor_ && state.find("horizontal_min") != std::string::npos) {
      if (state == "horizontal_min_0" || state == "horizontal_min_1") {
        if (state == pending_horizontal_min_state_) {
          horizontal_min_state_count_++;
          if (horizontal_min_state_count_ >= debounce_threshold_) {
            handleHorizontalMinState(state);
            horizontal_min_state_count_ = 0;
            pending_horizontal_min_state_ = "";
          }
        } else {
          pending_horizontal_min_state_ = state;
          horizontal_min_state_count_ = 1;
        }
      }
    }

    // ===================== WORK (cache vertical_min state only) =====================
    if (main_state_ == MainState::WORK && enable_zd_motor_ && state.find("vertical_min") != std::string::npos) {
      if (state == "vertical_min_0" || state == "vertical_min_1") {
        if (state == work_pending_vertical_min_state_) {
          work_vertical_min_state_count_++;
          if (work_vertical_min_state_count_ >= debounce_threshold_) {
            if (state != work_vertical_min_state_) {
              RCLCPP_INFO(this->get_logger(), "WORK: vertical_min state changed: '%s' -> '%s'",
                          work_vertical_min_state_.c_str(), state.c_str());
              work_vertical_min_state_ = state;
            }
            work_vertical_min_state_count_ = 0;
            work_pending_vertical_min_state_.clear();
          }
        } else {
          work_pending_vertical_min_state_ = state;
          work_vertical_min_state_count_ = 1;
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
        // Mark INIT vertical done on the homing-complete edge; don't block state transition on command return value.
        init_vertical_done_ = true;
        setZdMotorCommandRetry(motor_, rs485_interface::ZdMotor::ControlCommand::STOP);
      } else {
        RCLCPP_INFO(this->get_logger(), "ZD motor reversing to find home (vertical_max_0) at %d RPM", reverse_speed_rpm_);
        setZdMotorSpeedRetry(motor_, reverse_speed_rpm_);
        setZdMotorCommandRetry(motor_, rs485_interface::ZdMotor::ControlCommand::REVERSE);
      }
    } else if (state == "vertical_max_1") {
      RCLCPP_INFO(this->get_logger(), "ZD motor hit microswitch, forwarding to leave (vertical_max_1) at %d RPM", forward_speed_rpm_);
      setZdMotorSpeedRetry(motor_, forward_speed_rpm_);
      setZdMotorCommandRetry(motor_, rs485_interface::ZdMotor::ControlCommand::FORWARD);
    }
  }

  void handleVerticalMinState(const std::string & state)
  {
    if (state != "vertical_min_0" && state != "vertical_min_1") {
      RCLCPP_DEBUG(this->get_logger(), "Ignoring invalid vertical MIN state: '%s'", state.c_str());
      return;
    }

    if (state == current_vertical_min_state_) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Vertical MIN state changed: '%s' -> '%s'",
                current_vertical_min_state_.c_str(), state.c_str());

    current_vertical_min_state_ = state;

    if (!motor_) {
      RCLCPP_WARN(this->get_logger(), "ZD motor object not available; cannot execute PRE vertical MIN action");
      return;
    }

    // PRE rule: go to MIN using FORWARD, when hit MIN_1, STOP then REVERSE back to MIN_0, then STOP and mark done.
    // Direction note (user confirmed): ZD goes to MIN using FORWARD.
    if (state == "vertical_min_1") {
      if (!pre_vertical_reversing_back_) {
        // First time reaching MIN_1: STOP, then start reversing back
        RCLCPP_INFO(this->get_logger(), "ZD motor reached MIN microswitch, stopping then reversing back");
        setZdMotorCommandRetry(motor_, rs485_interface::ZdMotor::ControlCommand::STOP);
        pre_vertical_reversing_back_ = true;
        // Start reversing back to MIN_0
        setZdMotorSpeedRetry(motor_, reverse_speed_rpm_);
        setZdMotorCommandRetry(motor_, rs485_interface::ZdMotor::ControlCommand::REVERSE);
      }
    } else {  // vertical_min_0
      if (pre_vertical_reversing_back_) {
        // Reached MIN_0 after reversing back: STOP and mark done
        RCLCPP_INFO(this->get_logger(), "ZD motor reached MIN_0 after reversing back, stopping and marking PRE done");
        setZdMotorCommandRetry(motor_, rs485_interface::ZdMotor::ControlCommand::STOP);
        pre_vertical_done_ = true;
        pre_vertical_reversing_back_ = false;
      } else {
        // Initial state: moving to MIN using FORWARD
        RCLCPP_INFO(this->get_logger(), "ZD motor moving to MIN (FORWARD) at %d RPM", forward_speed_rpm_);
        setZdMotorSpeedRetry(motor_, forward_speed_rpm_);
        setZdMotorCommandRetry(motor_, rs485_interface::ZdMotor::ControlCommand::FORWARD);
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
        setServoEnableRetry(servo_motor_, rs485_interface::LcServoMotor::EnableState::ENABLE);
        // Mark INIT horizontal done on the homing-complete edge; don't block state transition on command return value.
        init_horizontal_done_ = true;
        setServoDirectionRetry(servo_motor_, rs485_interface::LcServoMotor::Direction::STOP);
      } else {
        RCLCPP_INFO(this->get_logger(), "LC Servo motor forwarding to find home (horizontal_max_0) at %.1f RPM", servo_forward_speed_rpm_);
        setServoEnableRetry(servo_motor_, rs485_interface::LcServoMotor::EnableState::ENABLE);
        setServoSpeedRetry(servo_motor_, servo_forward_speed_rpm_);
        setServoDirectionRetry(servo_motor_, rs485_interface::LcServoMotor::Direction::FORWARD);
      }
    } else if (state == "horizontal_max_1") {
      RCLCPP_INFO(this->get_logger(), "LC Servo motor hit microswitch, reversing to leave (horizontal_max_1) at %.1f RPM", servo_reverse_speed_rpm_);
      setServoEnableRetry(servo_motor_, rs485_interface::LcServoMotor::EnableState::ENABLE);
      setServoSpeedRetry(servo_motor_, servo_reverse_speed_rpm_);
      setServoDirectionRetry(servo_motor_, rs485_interface::LcServoMotor::Direction::REVERSE);
    }
  }

  void handleHorizontalMinState(const std::string & state)
  {
    if (state != "horizontal_min_0" && state != "horizontal_min_1") {
      RCLCPP_DEBUG(this->get_logger(), "Ignoring invalid horizontal MIN state: '%s'", state.c_str());
      return;
    }

    if (state == current_horizontal_min_state_) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Horizontal MIN state changed: '%s' -> '%s'",
                current_horizontal_min_state_.c_str(), state.c_str());

    current_horizontal_min_state_ = state;

    if (!servo_motor_) {
      RCLCPP_WARN(this->get_logger(), "LC Servo motor object not available; cannot execute PRE horizontal MIN action");
      return;
    }

    // PRE rule: go to MIN using REVERSE, when hit MIN_1, STOP then FORWARD back to MIN_0, then STOP and mark done.
    // Direction note (user confirmed): Servo goes to MIN using REVERSE.
    setServoEnableRetry(servo_motor_, rs485_interface::LcServoMotor::EnableState::ENABLE);
    if (state == "horizontal_min_1") {
      if (!pre_horizontal_reversing_back_) {
        // First time reaching MIN_1: STOP, then start forwarding back
        RCLCPP_INFO(this->get_logger(), "LC Servo reached MIN microswitch, stopping then forwarding back");
        setServoDirectionRetry(servo_motor_, rs485_interface::LcServoMotor::Direction::STOP);
        pre_horizontal_reversing_back_ = true;
        // Start forwarding back to MIN_0
        setServoSpeedRetry(servo_motor_, servo_forward_speed_rpm_);
        setServoDirectionRetry(servo_motor_, rs485_interface::LcServoMotor::Direction::FORWARD);
      }
    } else {  // horizontal_min_0
      if (pre_horizontal_reversing_back_) {
        // Reached MIN_0 after forwarding back: STOP and mark done
        RCLCPP_INFO(this->get_logger(), "LC Servo reached MIN_0 after forwarding back, stopping and marking PRE done");
        setServoDirectionRetry(servo_motor_, rs485_interface::LcServoMotor::Direction::STOP);
        pre_horizontal_done_ = true;
        pre_horizontal_reversing_back_ = false;
      } else {
        // Initial state: moving to MIN using REVERSE
        RCLCPP_INFO(this->get_logger(), "LC Servo moving to MIN (REVERSE) at %.1f RPM", servo_reverse_speed_rpm_);
        setServoSpeedRetry(servo_motor_, servo_reverse_speed_rpm_);
        setServoDirectionRetry(servo_motor_, rs485_interface::LcServoMotor::Direction::REVERSE);
      }
    }
  }

  bool enable_zd_motor_;
  bool enable_servo_motor_;
  std::shared_ptr<rs485_interface::RS485Client> rs485_client_;
  std::shared_ptr<rs485_interface::ZdMotor> motor_;
  std::shared_ptr<rs485_interface::ZdMotor> motor_work_;
  std::shared_ptr<rs485_interface::RS485Client> servo_rs485_client_;
  std::shared_ptr<rs485_interface::LcServoMotor> servo_motor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr microswitch_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_machine_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr microswitch_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vertical_max_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vertical_min_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr horizontal_max_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr horizontal_min_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zd_motor_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr servo_motor_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_work_state_pub_;

  // Subscribers for manual control
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_machine_set_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr confirm_wait_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr zd_motor_manual_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr zd_motor_manual_direction_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_motor_manual_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr servo_motor_manual_direction_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_motor_manual_enable_sub_;

  // Status publishing timer
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Manual mode and state tracking
  bool manual_mode_;
  int zd_motor_current_speed_;
  std::string zd_motor_current_direction_;
  double servo_motor_current_speed_;
  std::string servo_motor_current_direction_;
  bool servo_motor_enabled_;
  int motor_work_current_speed_;
  std::string motor_work_current_direction_;
  std::string last_microswitch_status_;
  
  int reverse_speed_rpm_;
  int forward_speed_rpm_;
  int work_speed_rpm_;
  int work_lift_speed_rpm_;
  double servo_forward_speed_rpm_;
  double servo_reverse_speed_rpm_;
  std::string current_vertical_state_;
  std::string current_horizontal_state_;
  std::string pending_vertical_state_;
  std::string pending_horizontal_state_;
  int vertical_state_count_;
  int horizontal_state_count_;
  int debounce_threshold_;

  // PRE: MIN microswitch debounce/state
  std::string current_vertical_min_state_;
  std::string current_horizontal_min_state_;
  std::string pending_vertical_min_state_;
  std::string pending_horizontal_min_state_;
  int vertical_min_state_count_{0};
  int horizontal_min_state_count_{0};

  // INIT completion flags (used for INIT -> PRE auto transition)
  bool init_vertical_done_{false};
  bool init_horizontal_done_{false};

  // PRE completion flags (used for PRE -> WORK auto transition)
  bool pre_vertical_done_{false};
  bool pre_horizontal_done_{false};
  bool pre_vertical_reversing_back_{false};  // Flag: reached MIN_1, now reversing back to MIN_0
  bool pre_horizontal_reversing_back_{false};  // Flag: reached MIN_1, now forwarding back to MIN_0

  // WORK one-shot command flag (avoid spamming RS485)
  bool motor_work_started_{false};

  // WORK: vertical_min-triggered delayed forward action
  std::string work_vertical_min_state_;
  std::string work_pending_vertical_min_state_;
  int work_vertical_min_state_count_{0};
  bool work_waiting_for_vertical_min_{false};
  bool work_forwarding_to_vertical_min_{false};
  rclcpp::Time work_wait_start_time_{0, 0, RCL_ROS_TIME};
  bool work_motor_forward_started_{false};

  // WORK: motor_work_ stop delay after vertical_min_0
  bool work_motor_work_waiting_stop_{false};
  rclcpp::Time work_motor_work_stop_time_{0, 0, RCL_ROS_TIME};

  MainState main_state_;
  MainState last_logged_state_;
  rclcpp::TimerBase::SharedPtr state_timer_;
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
