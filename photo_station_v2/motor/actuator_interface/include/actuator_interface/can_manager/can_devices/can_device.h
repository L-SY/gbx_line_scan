// can_device.h - ROS2 version

#pragma once

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>

// Forward declarations
namespace can_interface {
  struct CanFrameStamp;
}

namespace device {

enum class DeviceType {
  only_read,
  only_write,
  read_write
};

class CanDevice {
public:
  CanDevice(const std::string& name,
            const std::string& bus,
            int id,
            const std::string& model,
            DeviceType type,
            const YAML::Node& config = YAML::Node())
      : name_(name)
        , bus_(bus)
        , id_(id)
        , model_(model)
        , type_(type)
        , is_halted_(true)
        , config_(config)
        , seq_(0)
        , frequency_(0.0)
        , last_timestamp_(rclcpp::Clock().now())
        , logger_(rclcpp::get_logger(std::string("can_device_") + name))
  {
  }

  CanDevice(const std::string& name,
            const std::string& bus,
            int id,
            const std::string& model,
            DeviceType type)
      : CanDevice(name, bus, id, model, type, YAML::Node())
  {
  }

  virtual ~CanDevice() = default;

  virtual can_frame start() = 0;
  virtual can_frame close() = 0;
  virtual void read(const can_interface::CanFrameStamp& frameStamp) = 0;
  virtual void readBuffer(const std::vector<can_interface::CanFrameStamp>& buffer) = 0;
  virtual can_frame write() = 0;

  std::string getName() const { return name_; }
  std::string getBus() const { return bus_; }
  int getId() const { return id_; }
  DeviceType getType() const { return type_; }
  std::string getModel() const { return model_; }
  bool isValid() const { return is_halted_; }
  void setValid(bool valid) { is_halted_ = valid; }

  void updateFrequency(const rclcpp::Time& stamp) {
    double dt = (stamp - last_timestamp_).seconds();
    if (dt > 0.0) {
      frequency_ = 1.0 / dt;
    }
    last_timestamp_ = stamp;
  }

  uint32_t getSeq() const { return seq_; }
  double getFrequency() const { return frequency_; }
  rclcpp::Time getLastTimestamp() const { return last_timestamp_; }

protected:
  std::string name_;
  std::string bus_;
  int id_;
  DeviceType type_;
  std::string model_;
  bool is_halted_;
  YAML::Node config_;

  uint32_t seq_;
  double frequency_;
  rclcpp::Time last_timestamp_;
  rclcpp::Logger logger_;
};

} // namespace device 