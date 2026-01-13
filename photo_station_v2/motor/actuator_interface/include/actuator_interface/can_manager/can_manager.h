//
// CAN Manager for ROS2
//

#pragma once

#include "can_devices/can_device.h"
#include "can_devices/can_actuator.h"
#include "can_devices/rm_actuator.h"
#include "can_interface/can_bus.h"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <unordered_map>
#include <vector>
#include <thread>
#include <mutex>

namespace device {

class CanManager {
public:
  explicit CanManager(std::shared_ptr<rclcpp::Node> node);
  ~CanManager();

  bool init();

  bool addCanBus(const std::string& bus_name, int thread_priority);

  bool addDevice(const std::string& name, const std::string& bus, int id, const std::string& model,
                 const YAML::Node& config = YAML::Node());

  bool start();

  void close();

  void read();

  void write();

  const std::unordered_map<std::string, std::shared_ptr<CanDevice>>& getDevices() const {
    return devices_;
  }

  const std::vector<std::string>& getActuatorNames() const {
    return actuator_names_;
  }

  const std::unordered_map<std::string, std::shared_ptr<CanActuator>>& getActuatorDevices() const {
    return actuator_devices_;
  }

  void delayMicroseconds(unsigned int us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
  }

  // Configuration loading
  bool loadConfigFromFile(const std::string& config_file);

private:
  std::vector<can_interface::CanBus*> can_buses_{};

  std::unordered_map<std::string, std::shared_ptr<CanDevice>> devices_;

  std::vector<std::string> actuator_names_;
  std::unordered_map<std::string, std::shared_ptr<CanActuator>> actuator_devices_;

  std::unordered_map<std::string, std::unordered_map<int, std::shared_ptr<CanDevice>>> bus_devices_;

  bool running_;
  bool config_loaded_;
  std::vector<std::thread> read_threads_;
  std::mutex devices_mutex_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Logger logger_;

  bool loadBusConfig();
  bool loadDeviceConfig();
};

} // namespace device 