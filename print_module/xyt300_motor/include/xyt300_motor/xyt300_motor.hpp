#ifndef XYT300_MOTOR_HPP
#define XYT300_MOTOR_HPP

#include "xyt300_motor/modbus_client.hpp"

#include <cstdint>
#include <string>
#include <memory>

namespace xyt300_motor
{

/**
 * @brief XYT300 Motor driver using Modbus RTU protocol
 * 
 * This class implements the driver for XYT300 motor using
 * Modbus RTU protocol over RS485 interface.
 * 
 * Protocol specifications:
 * - Baud rate: 115200 (default)
 * - Data format: 8N1 (No parity, 8 data bits, 1 stop bit)
 * - Function codes: 0x03 (Read), 0x06 (Write Single)
 * 
 * Register map (to be adjusted according to actual protocol):
 * - 0x2000: Control command (0x01=forward, 0x02=reverse, 0x05=stop)
 * - 0x2001: Speed setting (RPM value)
 * - 0x2100: Status word (current speed, position, etc.)
 * - 0x0800: Slave address register
 */
class Xyt300Motor
{
public:
  /**
   * @brief Motor control commands
   */
  enum class ControlCommand : uint16_t
  {
    START = 0x0001,      // 启动 (write to 0x0050)
    FORWARD = 0x0002,    // 正转 (write to 0x0054)
    REVERSE = 0x0003,    // 反转 (write to 0x0056)
    STOP = 0x0000        // 停机 (write 0x0000 to forward register)
  };

  /**
   * @brief Constructor
   * @param modbus_client Shared pointer to MODBUS client
   * @param slave_address Slave device address (default: 1)
   * @param pulses_per_revolution Pulses per revolution (Pv) for speed calculation (default: 16)
   * @param gear_ratio Gear reduction ratio (default: 70)
   * @param max_output_rpm Maximum output shaft RPM for duty cycle mapping (default: 85)
   * 
   * Speed calculation formula:
   * Motor shaft RPM = (60 * pulse_frequency) / pulses_per_revolution
   * Output shaft RPM = Motor shaft RPM / gear_ratio
   */
  Xyt300Motor(
    std::shared_ptr<ModbusClient> modbus_client,
    uint8_t slave_address = 1,
    uint16_t pulses_per_revolution = 16,
    uint16_t gear_ratio = 70,
    uint16_t max_output_rpm = 85
  );

  /**
   * @brief Initialize the motor
   * @param skip_status_check If true, skip status word check (only verify communication)
   * @return true if successful, false otherwise
   */
  bool initialize(bool skip_status_check = false);

  /**
   * @brief Get device type
   * @return Device type string
   */
  std::string getDeviceType() const;

  /**
   * @brief Read current motor ID from slave address register
   * @param id Output parameter for the ID
   * @return true if successful, false otherwise
   */
  bool readMotorId(uint8_t & id);

  /**
   * @brief Set motor ID (slave address register)
   * @param new_id New ID to set (1-247)
   * @return true if successful, false otherwise
   * 
   * Two methods are supported:
   * 1. Function code 0x10: register address 0x00FA, data = 0x00XX (XX is device address)
   * 2. Function code 0x06: register address 0x4000, data = 0x00XX (XX is device address)
   * 
   * Note: This should only be done when motor is not running.
   */
  bool setMotorId(uint8_t new_id);

  /**
   * @brief Scan for motor IDs on the bus
   * @param found_ids Output vector to store found device IDs
   * @param start_address Starting address to scan (default: 1)
   * @param end_address Ending address to scan (default: 247)
   * @return true if at least one device was found, false otherwise
   * 
   * This function scans all possible slave addresses and tries to read
   * the device ID from each address. Found IDs are stored in found_ids.
   */
  bool scanMotorIds(std::vector<uint8_t> & found_ids, uint8_t start_address = 1, uint8_t end_address = 247);

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
   * @brief Write a register value without waiting for response (for initialization)
   * @param register_address Register address to write
   * @param value Value to write
   * @return true if successful, false otherwise
   */
  bool writeRegisterNoResponse(uint16_t register_address, uint16_t value);

  /**
   * @brief Set motor control command
   * @param command Control command (FORWARD, REVERSE, STOP, FAULT_RESET)
   * @return true if successful, false otherwise
   */
  bool setControlCommand(ControlCommand command);

  /**
   * @brief Set motor speed in RPM
   * @param speed_rpm Output shaft speed in RPM (after gear reduction)
   * @return true if successful, false otherwise
   * 
   * Converts output shaft RPM to PWM duty cycle (0-500),
   * then sends to device using protocol formula: send_value = 500 - duty_cycle
   * Maximum output shaft RPM is limited by max_output_rpm parameter.
   */
  bool setSpeedRPM(uint16_t speed_rpm);

  /**
   * @brief Read status word
   * @param status Output parameter for status word
   * @return true if successful, false otherwise
   */
  bool readStatusWord(uint16_t & status);

  /**
   * @brief Read current speed
   * @param speed_rpm Output parameter for speed in RPM (output shaft)
   * @return true if successful, false otherwise
   * 
   * Uses function code 0x04, register address 0x0020.
   * Reads pulse count (获取脉冲个数) and converts to RPM using:
   * - Motor shaft RPM = (60 * pulse_frequency) / pulses_per_revolution
   * - Output shaft RPM = Motor shaft RPM / gear_ratio
   * 
   * Note: Assumes the register returns pulse frequency (pulses per second).
   * If it returns pulse count in a different time window, adjust accordingly.
   */
  bool readCurrentSpeed(uint16_t & speed_rpm);

  /**
   * @brief Enable or disable motor
   * @param enable true to enable, false to disable
   * @return true if successful, false otherwise
   */
  bool setEnable(bool enable);

  /**
   * @brief Read DI (Digital Input) value
   * @param di_number DI number (typically 1-8 or 1-16)
   * @param value Output parameter for DI value (true = high, false = low)
   * @return true if successful, false otherwise
   */
  bool readDI(uint8_t di_number, bool & value);

  /**
   * @brief Read all DI values
   * @param di_values Output parameter for DI values (bitmap, bit 0 = DI1, bit 1 = DI2, etc.)
   * @return true if successful, false otherwise
   */
  bool readAllDI(uint16_t & di_values);

  /**
   * @brief Set DI configuration or force DI value (if supported)
   * @param di_number DI number (typically 1-8 or 1-16)
   * @param value Value to set
   * @return true if successful, false otherwise
   * 
   * Note: Not all motors support setting DI values. This may configure DI behavior instead.
   */
  bool setDI(uint8_t di_number, bool value);

  /**
   * @brief Get last error message
   * @return Error message string
   */
  std::string getLastError() const;

private:
  // Register addresses (from actual protocol documentation)
  static constexpr uint16_t REG_START = 0x0050;            // Start command
  static constexpr uint16_t REG_FORWARD = 0x0054;          // Forward command
  static constexpr uint16_t REG_REVERSE = 0x0056;          // Reverse command
  static constexpr uint16_t REG_SPEED_SETTING = 0x005A;    // Speed setting
  static constexpr uint16_t REG_READ_STATUS = 0x0010;      // Read status (function code 0x04)
  static constexpr uint16_t REG_READ_SPEED = 0x0020;      // Read speed (function code 0x04) - pulse count
  static constexpr uint16_t REG_SLAVE_ADDRESS_NORMAL = 0x0010;  // Slave address register (normal mode, function code 0x04)
  static constexpr uint16_t REG_SLAVE_ADDRESS_MODBUS = 0x000D;  // Slave address register (Modbus mode, function code 0x03)
  static constexpr uint16_t REG_SLAVE_ADDRESS_10H = 0x00FA;     // Slave address register (function code 0x10)
  static constexpr uint16_t REG_SLAVE_ADDRESS_06H = 0x4000;     // Slave address register (Modbus mode, function code 0x06)
  static constexpr uint16_t REG_SLAVE_ADDRESS = 0x0800;    // Slave address register (legacy, may not be correct)
  static constexpr uint16_t REG_CRC_INIT = 0x0000;        // CRC initialization register (set to 0xFFFF for init)

  std::shared_ptr<ModbusClient> modbus_client_;
  uint8_t slave_address_;
  std::string last_error_;
  bool speed_set_;  // Track if speed has been set
  uint16_t pulses_per_revolution_;  // Pv: pulses per revolution
  uint16_t gear_ratio_;  // Gear reduction ratio
  uint16_t max_output_rpm_;  // Maximum output shaft RPM for duty cycle mapping
  static constexpr uint16_t MAX_DUTY_CYCLE = 500;  // Maximum PWM duty cycle value
};

}  // namespace xyt300_motor

#endif  // XYT300_MOTOR_HPP
