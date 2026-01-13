#ifndef LC_STEPPER_MOTOR_HPP
#define LC_STEPPER_MOTOR_HPP

#include "rs485_interface/common/rs485_device_base.hpp"
#include "rs485_interface/common/rs485_client.hpp"

#include <cstdint>
#include <string>

namespace rs485_interface
{

/**
 * @brief LC Stepper Motor driver (CL57 compatible)
 * 
 * This class implements the driver for LC series stepper motors (CL57) using
 * custom RS485 protocol over serial interface.
 * 
 * Register addresses:
 * - Position control: 0x0037 (Write multiple registers, Function Code 0x10)
 * - Speed control: 0x0036 (Write single register, Function Code 0x06)
 * - State control: 0x004E (Write single register, Function Code 0x06)
 * 
 * Usage:
 * 1. Set state to start mode (enable motor)
 * 2. Set target position (mm)
 * 3. Set target speed (rpm)
 */
class LcStepperMotor : public RS485DeviceBase
{
public:
  /**
   * @brief Motor state options for CL57
   */
  enum class MotorState : uint16_t
  {
    START = 1,      // Start mode (enable motor)
    STOP = 0,       // Stop
    // Add other states as needed based on CL57 documentation
  };

  /**
   * @brief Constructor
   * @param rs485_client Shared pointer to RS485 client
   * @param slave_address Slave device address (default: 1)
   */
  LcStepperMotor(
    std::shared_ptr<RS485Client> rs485_client,
    uint8_t slave_address = 1
  );

  /**
   * @brief Initialize the motor
   * @return true if successful, false otherwise
   */
  bool initialize() override;

  /**
   * @brief Get device type
   * @return Device type string
   */
  std::string getDeviceType() const override;

  /**
   * @brief Get motor type enumeration
   * @return Motor type enum value
   */
  MotorType getMotorType() const override { return MotorType::LC_STEPPER_MOTOR; }

  /**
   * @brief Set motor state (start/stop mode)
   * @param state Motor state to set
   * @return true if successful, false otherwise
   * 
   * This should be called first to enable the motor before setting position/speed.
   */
  bool setState(MotorState state);

  /**
   * @brief Set target position
   * @param position Target position in millimeters (mm) or pulses
   * @return true if successful, false otherwise
   */
  bool setPosition(float position);

  /**
   * @brief Set target speed
   * @param speed Target speed in RPM
   * @return true if successful, false otherwise
   */
  bool setSpeed(float speed);

  /**
   * @brief Move to target position at specified speed
   * @param position Target position in mm or pulses
   * @param speed Movement speed in RPM
   * @return true if successful, false otherwise
   * 
   * This method sets speed first, then position to trigger movement.
   * Use this for position control applications.
   */
  bool moveTo(float position, float speed);

  /**
   * @brief Move relative distance at specified speed
   * @param distance Relative distance to move (positive = forward, negative = backward)
   * @param speed Movement speed in RPM
   * @return true if successful, false otherwise
   */
  bool moveRelative(float distance, float speed);

  /**
   * @brief Get current position (if supported by motor)
   * @param position Output parameter for current position
   * @return true if successful, false otherwise
   */
  bool getCurrentPosition(float & position);

private:
  float current_position_ = 0.0f;  // Track position locally
  static constexpr uint16_t REG_POSITION = 0x0037;  // Position register (multiple registers)
  static constexpr uint16_t REG_SPEED = 0x0036;     // Speed control register
  static constexpr uint16_t REG_STATE = 0x004E;     // State control register
};

}  // namespace rs485_interface

#endif  // LC_STEPPER_MOTOR_HPP

