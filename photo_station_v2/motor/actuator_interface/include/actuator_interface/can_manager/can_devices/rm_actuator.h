//
// RM Actuator for ROS2 - Simplified effort-only interface
//

#pragma once

#include "can_actuator.h"
#include <stdexcept>

namespace device {

/**
 * @brief RM series actuator implementation (RM2006, RM3508, etc.)
 * 
 * This class provides a simplified interface focused on effort control.
 * Higher-level control (position/velocity) should be implemented in ros2_control controllers.
 * 
 * Features:
 * - Raw CAN communication with RM actuators
 * - Multi-turn position tracking
 * - Effort (torque current) commands
 * - Position, velocity, effort state feedback
 */
class RmActuator : public CanActuator {
public:
  RmActuator(const std::string& name,
             const std::string& bus,
             int id,
             const std::string& motor_type,
             const YAML::Node& config);

  ~RmActuator() override = default;

  // Device lifecycle
  can_frame start() override;
  can_frame close() override;
  
  // Communication
  void read(const can_interface::CanFrameStamp& frameStamp) override;
  void readBuffer(const std::vector<can_interface::CanFrameStamp>& buffer) override;
  can_frame write() override;

  // Configuration
  ActuatorCoefficients getCoefficientsFor() const override;

private:
  // Configuration parameters
  double max_out_{10000};      // Maximum output value
  
  // Raw sensor values
  uint16_t q_raw_{0};          // Current encoder value
  uint16_t q_raw_last_{0};     // Previous encoder value
  int16_t qd_raw_{0};          // Velocity in RPM
  int16_t current_raw_{0};     // Current/torque value
  uint8_t temp_{0};            // Temperature
  
  // Multi-turn position tracking
  int64_t q_circle_{0};        // Circle count for multi-turn
  

};

} // namespace device 