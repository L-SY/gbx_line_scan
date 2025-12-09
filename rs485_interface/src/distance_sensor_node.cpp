#include "rs485_interface/distance_sensor.hpp"
#include "rs485_interface/modbus_client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <chrono>
#include <string>
#include <limits>

using namespace std::chrono_literals;

class DistanceSensorNode : public rclcpp::Node
{
public:
  DistanceSensorNode()
  : Node("distance_sensor_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("device_path", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<uint8_t>("slave_address", 1);
    this->declare_parameter<double>("publish_rate", 10.0);
    this->declare_parameter<std::string>("frame_id", "distance_sensor");
    this->declare_parameter<double>("min_range", 0.0);
    this->declare_parameter<double>("max_range", 500.0);
    this->declare_parameter<double>("field_of_view", 0.1);
    this->declare_parameter<int>("timeout_ms", 1000);

    // Get parameters
    std::string device_path = this->get_parameter("device_path").as_string();
    int baud_rate = this->get_parameter("baud_rate").as_int();
    uint8_t slave_address = static_cast<uint8_t>(this->get_parameter("slave_address").as_int());
    double publish_rate = this->get_parameter("publish_rate").as_double();
    std::string frame_id = this->get_parameter("frame_id").as_string();
    double min_range = this->get_parameter("min_range").as_double();
    double max_range = this->get_parameter("max_range").as_double();
    double field_of_view = this->get_parameter("field_of_view").as_double();
    int timeout_ms = this->get_parameter("timeout_ms").as_int();

    // Convert baud rate
    rs485_interface::ModbusClient::BaudRate modbus_baud_rate;
    switch (baud_rate) {
      case 9600:
        modbus_baud_rate = rs485_interface::ModbusClient::BaudRate::BAUD_9600;
        break;
      case 19200:
        modbus_baud_rate = rs485_interface::ModbusClient::BaudRate::BAUD_19200;
        break;
      case 38400:
        modbus_baud_rate = rs485_interface::ModbusClient::BaudRate::BAUD_38400;
        break;
      case 57600:
        modbus_baud_rate = rs485_interface::ModbusClient::BaudRate::BAUD_57600;
        break;
      case 115200:
        modbus_baud_rate = rs485_interface::ModbusClient::BaudRate::BAUD_115200;
        break;
      case 256000:
        modbus_baud_rate = rs485_interface::ModbusClient::BaudRate::BAUD_256000;
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Unsupported baud rate %d, using 115200", baud_rate);
        modbus_baud_rate = rs485_interface::ModbusClient::BaudRate::BAUD_115200;
    }

    // Create MODBUS client
    modbus_client_ = std::make_shared<rs485_interface::ModbusClient>(
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

    // Create distance sensor
    distance_sensor_ = std::make_unique<rs485_interface::DistanceSensor>(
      modbus_client_,
      slave_address
    );

    // Initialize sensor
    RCLCPP_INFO(this->get_logger(), "Initializing distance sensor...");
    if (!distance_sensor_->initialize()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to initialize sensor: %s",
        distance_sensor_->getLastError().c_str()
      );
      throw std::runtime_error("Failed to initialize sensor");
    }

    RCLCPP_INFO(this->get_logger(), "Distance sensor initialized successfully");

    // Create publishers
    range_pub_ = this->create_publisher<sensor_msgs::msg::Range>("distance/range", 10);
    distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("distance/value", 10);

    // Create services
    zero_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "distance/zero",
      std::bind(&DistanceSensorNode::zeroCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Create timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&DistanceSensorNode::timerCallback, this)
    );

    // Store parameters
    frame_id_ = frame_id;
    min_range_ = min_range;
    max_range_ = max_range;
    field_of_view_ = field_of_view;

    RCLCPP_INFO(this->get_logger(), "Distance sensor node started");
  }

  ~DistanceSensorNode()
  {
    if (modbus_client_) {
      modbus_client_->close();
    }
  }

private:
  void timerCallback()
  {
    double distance;
    if (!distance_sensor_->readDistance(distance)) {
      // Check if it's an invalid measurement (weak reflection, out of range)
      std::string error = distance_sensor_->getLastError();
      if (error.find("Invalid measurement") != std::string::npos) {
        // Publish NaN for invalid measurements so downstream nodes can detect it
        std_msgs::msg::Float64 distance_msg;
        distance_msg.data = std::numeric_limits<double>::quiet_NaN();
        distance_pub_->publish(distance_msg);

        sensor_msgs::msg::Range range_msg;
        range_msg.header.stamp = this->now();
        range_msg.header.frame_id = frame_id_;
        range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        range_msg.field_of_view = field_of_view_;
        range_msg.min_range = min_range_;
        range_msg.max_range = max_range_;
        range_msg.range = std::numeric_limits<double>::quiet_NaN();
        range_pub_->publish(range_msg);

        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "Invalid measurement (weak reflection or out of range), publishing NaN"
        );
      } else {
        // Communication error
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,
          "Failed to read distance: %s",
          error.c_str()
        );
      }
      return;
    }

    // Publish as Float64
    std_msgs::msg::Float64 distance_msg;
    distance_msg.data = distance;
    distance_pub_->publish(distance_msg);

    // Publish as Range
    sensor_msgs::msg::Range range_msg;
    range_msg.header.stamp = this->now();
    range_msg.header.frame_id = frame_id_;
    range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    range_msg.field_of_view = field_of_view_;
    range_msg.min_range = min_range_;
    range_msg.max_range = max_range_;
    range_msg.range = distance;
    range_pub_->publish(range_msg);
  }

  void zeroCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;  // Unused

    if (!distance_sensor_->zero()) {
      response->success = false;
      response->message = "Failed to zero: " + distance_sensor_->getLastError();
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    } else {
      response->success = true;
      response->message = "Zeroing completed successfully";
      RCLCPP_INFO(this->get_logger(), "Sensor zeroed");
    }
  }

  std::shared_ptr<rs485_interface::ModbusClient> modbus_client_;
  std::unique_ptr<rs485_interface::DistanceSensor> distance_sensor_;

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_;
  double min_range_;
  double max_range_;
  double field_of_view_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DistanceSensorNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("distance_sensor_node"), "Exception: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

