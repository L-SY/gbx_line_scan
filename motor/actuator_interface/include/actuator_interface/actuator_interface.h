//
// Actuator Interface for ROS2 - Generic actuator management layer
//

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include "actuator_interface/can_manager/can_manager.h"

namespace actuator_interface {

/**
 * @brief Generic actuator interface for CAN-based motor control
 * 
 * This class provides a hardware abstraction layer for managing multiple
 * CAN-based actuators. It handles:
 * - Dynamic actuator configuration from YAML
 * - Real-time state reading and command writing
 * - Extensible actuator types (RM series, etc.)
 * - Thread-safe operation
 */
class ActuatorInterface {
public:
  ActuatorInterface();
  ~ActuatorInterface() = default;

  // Initialization
  bool init(std::shared_ptr<rclcpp::Node> node);
  
  // Control loop functions
  void read(const rclcpp::Time& time, const rclcpp::Duration& period);
  void write(const rclcpp::Time& time, const rclcpp::Duration& period);
  
  // Configuration
  bool configure();
  bool activate();
  bool deactivate();
  
  // Interface getters - compatible with ros2_control
  std::vector<double*> getJointPositionPtrs();
  std::vector<double*> getJointVelocityPtrs();
  std::vector<double*> getJointEffortPtrs();
  std::vector<double*> getJointPositionCommandPtrs();
  std::vector<double*> getJointVelocityCommandPtrs();
  std::vector<double*> getJointEffortCommandPtrs();
  
  // Dynamic configuration
  bool loadFromYamlFile(const std::string& config_file);
  void addJoint(const std::string& joint_name, const std::string& actuator_name);
  size_t getJointCount() const { return joint_names_.size(); }
  const std::vector<std::string>& getJointNames() const { return joint_names_; }

private:
  // CAN Manager
  std::shared_ptr<device::CanManager> can_manager_;
  
  // Joint states (dynamically sized)
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  
  // Joint commands (dynamically sized)
  std::vector<double> joint_position_commands_;
  std::vector<double> joint_velocity_commands_;
  std::vector<double> joint_effort_commands_;
  
  // Joint configuration (dynamically loaded)
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, std::string> joint_to_actuator_map_;
  
  // ROS2 infrastructure
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> can_node_;
  rclcpp::Logger logger_;
  
  // Configuration state
  std::string config_file_;
  bool configured_;
  bool activated_;
  
  // Helper functions
  bool loadConfiguration();
  bool setupJointMapping();
  bool initializeCanManager();
  void resizeJointArrays();
};

} // namespace actuator_interface 