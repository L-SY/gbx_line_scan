//
// CAN Manager implementation for ROS2
//

#include "actuator_interface/can_manager/can_manager.h"
#include "actuator_interface/can_manager/can_interface/socketcan.h"
#include <fstream>
#include <chrono>
#include <rclcpp/logging.hpp>

namespace device {

CanManager::CanManager(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
    , running_(false)
    , config_loaded_(false)
    , logger_(node->get_logger())
{
    RCLCPP_INFO(logger_, "Initializing CAN Manager");
}

CanManager::~CanManager() {
    close();
}

bool CanManager::init() {
    RCLCPP_INFO(logger_, "Initializing CAN Manager");
    
    if (!loadBusConfig()) {
        RCLCPP_ERROR(logger_, "Failed to load CAN bus configuration");
        return false;
    }

    if (!loadDeviceConfig()) {
        RCLCPP_ERROR(logger_, "Failed to load CAN device configuration");
        return false;
    }

    RCLCPP_INFO(logger_, "CAN Manager initialization completed successfully");
    return true;
}

bool CanManager::loadBusConfig() {
    // Bus configuration is now loaded from YAML file in loadConfigFromFile
    // This method is kept for compatibility but will use default if no config file
    
    if (!config_loaded_) {
        RCLCPP_WARN(logger_, "No config file loaded, using default CAN bus: can0");
        return addCanBus("can0", 95);
    }
    
    return true;  // Configuration already loaded from YAML
}

bool CanManager::loadDeviceConfig() {
    // Device configuration is now loaded from YAML file in loadConfigFromFile
    // This method is kept for compatibility
    
    if (!config_loaded_) {
        RCLCPP_ERROR(logger_, "No config file loaded, cannot load device configuration");
        return false;
    }
    
    return true;  // Configuration already loaded from YAML
}

bool CanManager::loadConfigFromFile(const std::string& config_file) {
    try {
        RCLCPP_INFO(logger_, "Loading configuration from file: %s", config_file.c_str());
        
        YAML::Node root = YAML::LoadFile(config_file);
        
        if (!root["actuator_interface_hw"]) {
            RCLCPP_ERROR(logger_, "No actuator_interface_hw section found in config file");
            return false;
        }
        
        YAML::Node hw_config = root["actuator_interface_hw"];
        
        // Load bus configuration
        if (hw_config["buses"]) {
            for (const auto& bus_node : hw_config["buses"]) {
                std::string bus_name = bus_node["name"].as<std::string>();
                int priority = bus_node["priority"] ? bus_node["priority"].as<int>() : 95;
                
                if (!addCanBus(bus_name, priority)) {
                    RCLCPP_ERROR(logger_, "Failed to add CAN bus: %s", bus_name.c_str());
                    return false;
                }
                RCLCPP_INFO(logger_, "Added CAN bus: %s with priority %d", bus_name.c_str(), priority);
            }
        }
        
        // Load device configuration
        if (hw_config["devices"]) {
            for (const auto& device_node : hw_config["devices"]) {
                std::string name = device_node["name"].as<std::string>();
                std::string bus = device_node["bus"].as<std::string>();
                int id = device_node["id"].as<int>();
                std::string model = device_node["model"].as<std::string>();
                
                YAML::Node config = device_node["config"];
                
                if (!addDevice(name, bus, id, model, config)) {
                    RCLCPP_ERROR(logger_, "Failed to add device: %s", name.c_str());
                    return false;
                }
                RCLCPP_INFO(logger_, "Added device: %s", name.c_str());
            }
        }
        
        config_loaded_ = true;
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception loading config file: %s", e.what());
        return false;
    }
}

bool CanManager::addCanBus(const std::string& bus_name, int thread_priority) {
  can_buses_.push_back(new can_interface::CanBus(bus_name, thread_priority));
  bus_devices_[bus_name] = std::unordered_map<int, std::shared_ptr<CanDevice>>();
  return true;
}

bool CanManager::addDevice(const std::string& name, const std::string& bus, int id, 
                          const std::string& model, const YAML::Node& config) {
    try {
        std::shared_ptr<CanDevice> device;
        
        // Create device based on model type - extensible for any RM motor
        if (model.find("RM") != std::string::npos) {
            device = std::make_shared<RmActuator>(name, bus, id, model, config);
            
            // Add to actuator lists
            auto actuator = std::dynamic_pointer_cast<CanActuator>(device);
            if (actuator) {
                actuator_devices_[name] = actuator;
                actuator_names_.push_back(name);
            }
        } else {
            RCLCPP_ERROR(logger_, "Unsupported device model: %s. Currently supported: RM2006, RM3508, etc.", model.c_str());
            return false;
        }
        
        if (!device) {
            RCLCPP_ERROR(logger_, "Failed to create device: %s", name.c_str());
            return false;
        }
        
        // Add to general device list
        devices_[name] = device;
        
        // Add to bus device mapping
        bus_devices_[bus][id] = device;
        
        RCLCPP_INFO(logger_, "Successfully added device: %s (model: %s)", name.c_str(), model.c_str());
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception adding device %s: %s", name.c_str(), e.what());
        return false;
    }
}

bool CanManager::start() {
  std::lock_guard<std::mutex> lock(devices_mutex_);
  bool all_success = true;

  for (int i = 0; i < 5; i++) {
    RCLCPP_INFO(logger_, "Starting devices, attempt %d of 5", i+1);

    for (auto* can_bus : can_buses_) {
      if (!can_bus)
        continue;

      const std::string& bus_name = can_bus->getDeviceName();
      auto bus_it = bus_devices_.find(bus_name);

      if (bus_it != bus_devices_.end()) {
        for (const auto& device_pair : bus_it->second) {
          can_frame frame = device_pair.second->start();
          if (frame.can_dlc == 0) {
            continue;
          }
          delayMicroseconds(100000);
          can_bus->write(&frame);
        }
      }
    }
  }

  RCLCPP_INFO(logger_, "All devices start command sent 5 times!");
  running_ = true;
  return all_success;
}

void CanManager::close() {
  std::lock_guard<std::mutex> lock(devices_mutex_);

  for (int i = 0; i < 5; i++) {
    for (auto* can_bus : can_buses_) {
      if (!can_bus)
        continue;

      const std::string& bus_name = can_bus->getDeviceName();
      auto bus_it = bus_devices_.find(bus_name);

      if (bus_it != bus_devices_.end()) {
        for (const auto& device_pair : bus_it->second) {
          can_frame frame = device_pair.second->close();
          if (frame.can_dlc == 0) {
            continue;
          }
          delayMicroseconds(100000);
          can_bus->write(&frame);
        }
      }
    }
  }
  
  running_ = false;
}

void CanManager::read() {
  std::lock_guard<std::mutex> lock(devices_mutex_);
  
  for (auto* can_bus : can_buses_) {
    if (!can_bus) continue;

    const auto& read_buffer = can_bus->getReadBuffer();
    const std::string& bus_name = can_bus->getDeviceName();

    auto bus_it = bus_devices_.find(bus_name);
    if (bus_it == bus_devices_.end())
      continue;

    for (auto& device_pair : bus_it->second) {
      if (device_pair.second->getType() != DeviceType::only_write) {
        device_pair.second->readBuffer(read_buffer);
      }
    }

    can_bus->read();
  }
}

void CanManager::write() {
    if (!running_) {
        return;
    }
    
    try {
        // RM motor special handling - merge multiple motors into single frames
        for (auto* can_bus : can_buses_) {
            if (!can_bus) continue;

            const std::string& bus_name = can_bus->getDeviceName();
            auto bus_it = bus_devices_.find(bus_name);
            if (bus_it == bus_devices_.end()) continue;
            
            can_frame rm_frame0_{}, rm_frame1_{};
            rm_frame0_.can_id  = 0x200;
            rm_frame0_.can_dlc = 8;
            rm_frame1_.can_id  = 0x1FF;
            rm_frame1_.can_dlc = 8;
            bool has_write_frame0 = false, has_write_frame1 = false;

            for (const auto& device_pair : bus_it->second) {
                auto device_ptr = device_pair.second;
                if (device_ptr->getType() == DeviceType::only_read) continue;

                can_frame frame = device_ptr->write();
                if (frame.can_id == 0x200) {
                    for (int i = 0; i < 8; ++i) {
                        if (frame.data[i] != 0) {
                            rm_frame0_.data[i] = frame.data[i];
                        }
                        has_write_frame0 = true;
                    }
                }
                else if (frame.can_id == 0x1FF) {
                    for (int i = 0; i < 8; ++i) {
                        if (frame.data[i] != 0) {
                            rm_frame1_.data[i] = frame.data[i];
                        }
                        has_write_frame1 = true;
                    }
                }
                else {
                    can_bus->write(&frame);
                }
            }

            if (has_write_frame0) {
                can_bus->write(&rm_frame0_);
            }
            if (has_write_frame1) {
                can_bus->write(&rm_frame1_);
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception in CAN write: %s", e.what());
    }
}

} // namespace device 