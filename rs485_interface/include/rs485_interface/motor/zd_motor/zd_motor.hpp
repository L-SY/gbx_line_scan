#ifndef ZD_MOTOR_HPP
#define ZD_MOTOR_HPP

#include "rs485_interface/motor/lc_servo_motor/rs485_device_base.hpp"
#include "rs485_interface/motor/lc_servo_motor/rs485_client.hpp"

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

namespace rs485_interface
{

/**
 * @brief ZD Motor driver using Modbus RTU protocol
 * 
 * This class implements the driver for ZD series motors using
 * Modbus RTU protocol over RS485 interface.
 * 
 * Protocol specifications:
 * - Baud rate: 19200 (default)
 * - Data format: N-8-1 (No parity, 8 data bits, 1 stop bit)
 * - Function codes: 0x03 (Read), 0x06 (Write Single), 0x10 (Write Multiple)
 * - Register addresses: 0x2000-0x30ff
 * 
 * Register map:
 * - 0x2000: Control command (0x01=forward, 0x02=reverse, 0x05=stop, 0x07=fault reset)
 * - 0x2001: Speed setting (RPM value, e.g., 0x0BB8 = 3000 RPM)
 * - 0x2100: Status word 1
 * - 0x2101: Status word 2
 * - F08.00: 485 slave address (1-247, 0=broadcast)
 */
class ZdMotor : public RS485DeviceBaseServo
{
public:
  /**
   * @brief Motor control commands
   */
  enum class ControlCommand : uint16_t
  {
    FORWARD = 0x01,      // 正转运行
    REVERSE = 0x02,      // 反转运行
    STOP = 0x05,         // 停机
    FAULT_RESET = 0x07   // 故障复位
  };

  /**
   * @brief Constructor
   * @param rs485_client Shared pointer to RS485 client
   * @param slave_address Slave device address (default: 1)
   */
  ZdMotor(
    std::shared_ptr<RS485ClientServo> rs485_client,
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
   * @brief Scan for motor ID by reading F08.00 register
   * @param rs485_client RS485 client instance
   * @param found_id Output parameter for found ID
   * @param start_id Starting ID to scan from (default: 0)
   * @param end_id Ending ID to scan to (default: 10)
   * @return true if motor found, false otherwise
   * 
   * Scans through ID range to find the actual motor ID by reading
   * the F08.00 register (485 slave address register).
   */
  static bool scanMotorId(
    std::shared_ptr<RS485ClientServo> rs485_client,
    uint8_t & found_id,
    uint8_t start_id = 0,
    uint8_t end_id = 10
  );

  /**
   * @brief Read current motor ID from F08.00 register
   * @param id Output parameter for the ID
   * @return true if successful, false otherwise
   */
  bool readMotorId(uint8_t & id);

  /**
   * @brief Set motor ID (F08.00 register)
   * @param new_id New ID to set (1-247, 0=broadcast)
   * @return true if successful, false otherwise
   * 
   * Note: This should only be done when motor is not running.
   */
  bool setMotorId(uint8_t new_id);

  /**
   * @brief Read a register value
   * @param register_address Register address to read
   * @param value Output parameter for the read value
   * @return true if successful, false otherwise
   */
  bool readRegister(uint16_t register_address, uint16_t & value);

  /**
   * @brief Write a register value
   * @param register_address Register address to write
   * @param value Value to write
   * @return true if successful, false otherwise
   */
  bool writeRegister(uint16_t register_address, uint16_t value);

  /**
   * @brief Set motor control command
   * @param command Control command (FORWARD, REVERSE, STOP, FAULT_RESET)
   * @return true if successful, false otherwise
   */
  bool setControlCommand(ControlCommand command);

  /**
   * @brief Set motor speed in RPM
   * @param speed_rpm Speed in RPM (e.g., 3000)
   * @return true if successful, false otherwise
   * 
   * Writes to register 0x2001
   */
  bool setSpeedRPM(uint16_t speed_rpm);

  /**
   * @brief Read status word 1
   * @param status Output parameter for status word 1
   * @return true if successful, false otherwise
   * 
   * Reads from register 0x2100
   */
  bool readStatusWord1(uint16_t & status);

  /**
   * @brief Read status word 2
   * @param status Output parameter for status word 2
   * @return true if successful, false otherwise
   * 
   * Reads from register 0x2101
   */
  bool readStatusWord2(uint16_t & status);

private:
  // Register addresses
  static constexpr uint16_t REG_CONTROL_CMD = 0x2000;    // Control command
  static constexpr uint16_t REG_SPEED_SETTING = 0x2001;   // Speed setting
  static constexpr uint16_t REG_STATUS_WORD1 = 0x2100;    // Status word 1
  static constexpr uint16_t REG_STATUS_WORD2 = 0x2101;   // Status word 2
  static constexpr uint16_t REG_SLAVE_ADDRESS = 0x0800;  // F08.00 = 0x0800 (485 slave address)
  static constexpr uint16_t REG_WRITE_ENABLE = 0x200E;   // 200EH: Write enable for F00-F10 group function codes
};

}  // namespace rs485_interface

#endif  // ZD_MOTOR_HPP

