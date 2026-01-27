#include "microswitch/serial_reader.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class MicroswitchNode : public rclcpp::Node
{
public:
  MicroswitchNode()
  : Node("microswitch_node")
  {
    this->declare_parameter<std::string>("device_path", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<double>("read_rate", 100.0);
    this->declare_parameter<int>("timeout_ms", 1000);

    std::string device_path = this->get_parameter("device_path").as_string();
    int baud_rate = this->get_parameter("baud_rate").as_int();
    double read_rate = this->get_parameter("read_rate").as_double();
    int timeout_ms = this->get_parameter("timeout_ms").as_int();

    serial_reader_ = std::make_unique<microswitch::SerialReader>(device_path, baud_rate);

    RCLCPP_INFO(this->get_logger(), "Opening serial port: %s at %d baud", 
                device_path.c_str(), baud_rate);
    
    if (!serial_reader_->open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s",
                   serial_reader_->getLastError().c_str());
      throw std::runtime_error("Failed to open serial port");
    }

    RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");

    data_pub_ = this->create_publisher<std_msgs::msg::String>("microswitch/data", 10);

    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / read_rate));
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&MicroswitchNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Microswitch node started");
  }

  ~MicroswitchNode()
  {
    if (serial_reader_) {
      serial_reader_->close();
    }
  }

private:
  void timerCallback()
  {
    std::vector<std::string> lines;
    
    if (serial_reader_->readAvailable(lines)) {
      for (const auto & line : lines) {
        std_msgs::msg::String msg;
        msg.data = line;
        data_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published: %s", line.c_str());
      }
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Failed to read from serial: %s",
                          serial_reader_->getLastError().c_str());
    }
  }

  std::unique_ptr<microswitch::SerialReader> serial_reader_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MicroswitchNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("microswitch_node"), "Exception: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
