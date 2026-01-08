#ifndef RS485_COMMON_DEVICE_BASE_HPP
#define RS485_COMMON_DEVICE_BASE_HPP

#include <memory>
#include <string>
#include <cstdint>

namespace rs485_interface
{

class RS485Client;

/**
 * @brief Unified base class for all RS485 motor devices
 * 
 * This abstract base class defines the common interface for all RS485 motor devices.
 * Derived classes (ZdMotor, LcServoMotor, LcStepperMotor) should implement
 * device-specific functionality while using the common RS485 communication
 * provided by RS485Client.
 * 
 * This unified base class replaces the separate RS485DeviceBase and 
 * RS485DeviceBaseServo classes.
 */
class RS485DeviceBase
{
public:
  /**
   * @brief Motor type enumeration for identification
   */
  enum class MotorType
  {
    ZD_MOTOR,         // ZD series motor (变频电机)
    LC_SERVO_MOTOR,   // LC series servo motor (伺服电机)
    LC_STEPPER_MOTOR  // LC series stepper motor (步进电机)
  };

  /**
   * @brief Constructor
   * @param rs485_client Shared pointer to RS485 client
   * @param slave_address Slave device address
   */
  RS485DeviceBase(std::shared_ptr<RS485Client> rs485_client, uint8_t slave_address);

  /**
   * @brief Virtual destructor
   */
  virtual ~RS485DeviceBase() = default;

  // Non-copyable
  RS485DeviceBase(const RS485DeviceBase &) = delete;
  RS485DeviceBase & operator=(const RS485DeviceBase &) = delete;

  /**
   * @brief Initialize the device
   * @return true if successful, false otherwise
   */
  virtual bool initialize() = 0;

  /**
   * @brief Get device name/type as string
   * @return Device name string
   */
  virtual std::string getDeviceType() const = 0;

  /**
   * @brief Get motor type enumeration
   * @return Motor type enum value
   */
  virtual MotorType getMotorType() const = 0;

  /**
   * @brief Get slave address
   * @return Slave address
   */
  uint8_t getSlaveAddress() const;

  /**
   * @brief Set slave address
   * @param address New slave address
   */
  void setSlaveAddress(uint8_t address);

  /**
   * @brief Get RS485 client
   * @return Shared pointer to RS485 client
   */
  std::shared_ptr<RS485Client> getRS485Client() const;

  /**
   * @brief Check if device is connected (RS485 client is open)
   * @return true if connected
   */
  bool isConnected() const;

  /**
   * @brief Get last error message
   * @return Error message string
   */
  std::string getLastError() const;

protected:
  /**
   * @brief Set last error message
   * @param error Error message
   */
  void setLastError(const std::string & error);

  std::shared_ptr<RS485Client> rs485_client_;
  uint8_t slave_address_;
  std::string last_error_;
};

}  // namespace rs485_interface

#endif  // RS485_COMMON_DEVICE_BASE_HPP

