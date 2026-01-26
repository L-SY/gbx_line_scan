#include "xyt300_motor/xyt300_motor.hpp"
#include "xyt300_motor/modbus_client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
// std_msgs/msg/bool.hpp and std_srvs/srv/set_bool.hpp removed - motor doesn't need enable/disable
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class Xyt300MotorNode : public rclcpp::Node
{
public:
  Xyt300MotorNode()
  : Node("xyt300_motor_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("device_path", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<uint8_t>("slave_address", 1);
    this->declare_parameter<double>("publish_rate", 10.0);
    this->declare_parameter<int>("timeout_ms", 1000);
    this->declare_parameter<uint16_t>("pulses_per_revolution", 16);
    this->declare_parameter<uint16_t>("gear_ratio", 70);
    this->declare_parameter<uint16_t>("max_output_rpm", 85);

    // Get parameters
    std::string device_path = this->get_parameter("device_path").as_string();
    int baud_rate = this->get_parameter("baud_rate").as_int();
    uint8_t slave_address = static_cast<uint8_t>(this->get_parameter("slave_address").as_int());
    double publish_rate = this->get_parameter("publish_rate").as_double();
    int timeout_ms = this->get_parameter("timeout_ms").as_int();
    uint16_t pulses_per_revolution = static_cast<uint16_t>(this->get_parameter("pulses_per_revolution").as_int());
    uint16_t gear_ratio = static_cast<uint16_t>(this->get_parameter("gear_ratio").as_int());
    uint16_t max_output_rpm = static_cast<uint16_t>(this->get_parameter("max_output_rpm").as_int());

    // Convert baud rate
    xyt300_motor::ModbusClient::BaudRate modbus_baud_rate;
    switch (baud_rate) {
      case 2400:
        modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_2400;
        break;
      case 4800:
        modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_4800;
        break;
      case 9600:
        modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_9600;
        break;
      case 19200:
        modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_19200;
        break;
      case 38400:
        modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_38400;
        break;
      case 57600:
        modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_57600;
        break;
      case 115200:
        modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_115200;
        break;
      case 256000:
        modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_256000;
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Unsupported baud rate %d, using 115200", baud_rate);
        modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_115200;
    }

    // Create MODBUS client
    modbus_client_ = std::make_shared<xyt300_motor::ModbusClient>(
      device_path,
      modbus_baud_rate,
      timeout_ms
    );

    // Open serial port
    RCLCPP_INFO(this->get_logger(), "Opening serial port: %s", device_path.c_str());
    if (!modbus_client_->open()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to open serial port: %s",
        modbus_client_->getLastError().c_str()
      );
      throw std::runtime_error("Failed to open serial port");
    }

    // Create motor driver
    motor_ = std::make_unique<xyt300_motor::Xyt300Motor>(
      modbus_client_,
      slave_address,
      pulses_per_revolution,
      gear_ratio,
      max_output_rpm
    );

    // Initialize motor
    RCLCPP_INFO(this->get_logger(), "Initializing motor...");
    if (!motor_->initialize()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to initialize motor: %s",
        motor_->getLastError().c_str()
      );
      throw std::runtime_error("Failed to initialize motor");
    }

    RCLCPP_INFO(this->get_logger(), "Motor initialized successfully");

    // Create publishers
    speed_pub_ = this->create_publisher<std_msgs::msg::Int16>("motor/speed", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::Int16>("motor/status", 10);
    // enabled_pub_ removed - motor doesn't need enable/disable

    // Create subscribers
    speed_cmd_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "motor/speed_cmd",
      10,
      std::bind(&Xyt300MotorNode::speedCmdCallback, this, std::placeholders::_1)
    );

    control_cmd_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "motor/control_cmd",
      10,
      std::bind(&Xyt300MotorNode::controlCmdCallback, this, std::placeholders::_1)
    );

    // Create services
    // enable_srv_ removed - motor doesn't need enable/disable

    stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "motor/stop",
      std::bind(&Xyt300MotorNode::stopCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "motor/reset",
      std::bind(&Xyt300MotorNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Create timer for status publishing
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&Xyt300MotorNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "XYT300 motor node started");
  }

  ~Xyt300MotorNode()
  {
    if (modbus_client_) {
      modbus_client_->close();
    }
  }

private:
  void timerCallback()
  {
    // Read current speed only if speed has been set
    uint16_t speed_rpm = 0;
    if (motor_->readCurrentSpeed(speed_rpm)) {
      std_msgs::msg::Int16 speed_msg;
      speed_msg.data = static_cast<int16_t>(speed_rpm);
      speed_pub_->publish(speed_msg);
    }
    // Silently skip if speed hasn't been set yet (this is normal)

    // Read status word
    uint16_t status = 0;
    if (motor_->readStatusWord(status)) {
      std_msgs::msg::Int16 status_msg;
      status_msg.data = static_cast<int16_t>(status);
      status_pub_->publish(status_msg);
    }
  }

  void speedCmdCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    int16_t speed_cmd = msg->data;
    uint16_t speed_rpm = (speed_cmd < 0) ? 0 : static_cast<uint16_t>(speed_cmd);
    if (motor_->setSpeedRPM(speed_rpm)) {
      RCLCPP_INFO(this->get_logger(), "Speed set to %d RPM", speed_rpm);
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to set speed: %s",
        motor_->getLastError().c_str()
      );
    }
  }

  void controlCmdCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    int16_t cmd = msg->data;
    xyt300_motor::Xyt300Motor::ControlCommand command;

    switch (cmd) {
      case 1:
        command = xyt300_motor::Xyt300Motor::ControlCommand::FORWARD;
        break;
      case 2:
        command = xyt300_motor::Xyt300Motor::ControlCommand::REVERSE;
        break;
      case 5:
        command = xyt300_motor::Xyt300Motor::ControlCommand::STOP;
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Unknown control command: %d", cmd);
        return;
    }

    if (motor_->setControlCommand(command)) {
      RCLCPP_INFO(this->get_logger(), "Control command %d executed", cmd);
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to set control command: %s",
        motor_->getLastError().c_str()
      );
    }
  }

  // enableCallback removed - motor doesn't need enable/disable

  void stopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;  // Unused

    if (motor_->setControlCommand(xyt300_motor::Xyt300Motor::ControlCommand::STOP)) {
      response->success = true;
      response->message = "Motor stopped";
      RCLCPP_INFO(this->get_logger(), "Motor stopped");
    } else {
      response->success = false;
      response->message = "Failed to stop motor: " + motor_->getLastError();
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;  // Unused

    // Fault reset not supported by this motor protocol
    // Use STOP command instead
    if (motor_->setControlCommand(xyt300_motor::Xyt300Motor::ControlCommand::STOP)) {
      response->success = true;
      response->message = "Motor stopped (fault reset not supported)";
      RCLCPP_INFO(this->get_logger(), "Motor stopped (fault reset not supported)");
    } else {
      response->success = false;
      response->message = "Failed to stop motor: " + motor_->getLastError();
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  std::shared_ptr<xyt300_motor::ModbusClient> modbus_client_;
  std::unique_ptr<xyt300_motor::Xyt300Motor> motor_;

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr status_pub_;
  // enabled_pub_ removed - motor doesn't need enable/disable

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr speed_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr control_cmd_sub_;

  // enable_srv_ removed - motor doesn't need enable/disable
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Xyt300MotorNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("xyt300_motor_node"), "Exception: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
