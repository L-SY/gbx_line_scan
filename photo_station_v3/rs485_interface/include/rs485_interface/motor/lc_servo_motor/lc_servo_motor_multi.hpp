#ifndef LC_SERVO_MOTOR_MULTI_HPP
#define LC_SERVO_MOTOR_MULTI_HPP

#include "rs485_interface/motor/lc_servo_motor/lc_servo_motor.hpp"
#include "rs485_interface/common/rs485_client.hpp"

#include <vector>
#include <memory>
#include <string>
#include <cstdint>
#include <map>

namespace rs485_interface
{

/**
 * @brief Multi-motor manager for LC Servo Motors
 * 
 * This class manages multiple LC servo motors connected to the same RS485 bus.
 * All motors share the same serial port and communication parameters, but have
 * different slave addresses.
 */
class LcServoMotorMulti
{
public:
  /**
   * @brief Constructor
   * @param device_path Path to serial device (e.g., /dev/ttyUSB0)
   * @param baud_rate Baud rate for communication
   * @param parity Parity setting
   * @param timeout_ms Read timeout in milliseconds
   */
  LcServoMotorMulti(
    const std::string & device_path,
    RS485ClientServo::BaudRate baud_rate = RS485ClientServo::BaudRate::BAUD_19200,
    RS485ClientServo::Parity parity = RS485ClientServo::Parity::EVEN,
    int timeout_ms = 2000
  );

  /**
   * @brief Destructor
   */
  ~LcServoMotorMulti();

  /**
   * @brief Add a motor with specified address
   * @param address Slave address (1-247)
   * @param name Optional name for the motor (for identification)
   * @return true if successful, false otherwise
   */
  bool addMotor(uint8_t address, const std::string & name = "");

  /**
   * @brief Remove a motor by address
   * @param address Slave address
   * @return true if successful, false otherwise
   */
  bool removeMotor(uint8_t address);

  /**
   * @brief Get motor by address
   * @param address Slave address
   * @return Shared pointer to motor, or nullptr if not found
   */
  std::shared_ptr<LcServoMotor> getMotor(uint8_t address);

  /**
   * @brief Get all motor addresses
   * @return Vector of motor addresses
   */
  std::vector<uint8_t> getMotorAddresses() const;

  /**
   * @brief Get motor name by address
   * @param address Slave address
   * @return Motor name, or empty string if not found
   */
  std::string getMotorName(uint8_t address) const;

  /**
   * @brief Set motor name
   * @param address Slave address
   * @param name Motor name
   * @return true if successful, false otherwise
   */
  bool setMotorName(uint8_t address, const std::string & name);

  /**
   * @brief Open the RS485 connection
   * @return true if successful, false otherwise
   */
  bool open();

  /**
   * @brief Close the RS485 connection
   */
  void close();

  /**
   * @brief Check if connection is open
   * @return true if open, false otherwise
   */
  bool isOpen() const;

  /**
   * @brief Initialize all motors for speed control
   * @return Map of address to success status
   */
  std::map<uint8_t, bool> initializeAllMotors();

  /**
   * @brief Initialize a specific motor for speed control
   * @param address Slave address
   * @return true if successful, false otherwise
   */
  bool initializeMotor(uint8_t address);

  /**
   * @brief Enable all motors
   * @return Map of address to success status
   */
  std::map<uint8_t, bool> enableAllMotors();

  /**
   * @brief Disable all motors
   * @return Map of address to success status
   */
  std::map<uint8_t, bool> disableAllMotors();

  /**
   * @brief Set speed for all motors
   * @param speed_rpm Speed in RPM
   * @return Map of address to success status
   */
  std::map<uint8_t, bool> setSpeedAllMotors(double speed_rpm);

  /**
   * @brief Set direction for all motors
   * @param direction Direction (FORWARD, REVERSE, or STOP)
   * @return Map of address to success status
   */
  std::map<uint8_t, bool> setDirectionAllMotors(LcServoMotor::Direction direction);

  /**
   * @brief Stop all motors
   * @return Map of address to success status
   */
  std::map<uint8_t, bool> stopAllMotors();

  /**
   * @brief Get last error message
   * @return Error message string
   */
  std::string getLastError() const;

private:
  std::string device_path_;
  RS485ClientServo::BaudRate baud_rate_;
  RS485ClientServo::Parity parity_;
  int timeout_ms_;
  
  std::shared_ptr<RS485ClientServo> rs485_client_;
  std::map<uint8_t, std::shared_ptr<LcServoMotor>> motors_;
  std::map<uint8_t, std::string> motor_names_;
  std::string last_error_;
};

}  // namespace rs485_interface

#endif  // LC_SERVO_MOTOR_MULTI_HPP


