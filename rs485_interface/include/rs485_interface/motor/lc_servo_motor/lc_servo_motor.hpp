#ifndef LC_SERVO_MOTOR_HPP
#define LC_SERVO_MOTOR_HPP

#include "rs485_interface/motor/lc_servo_motor/rs485_device_base.hpp"
#include "rs485_interface/motor/lc_servo_motor/rs485_client.hpp"

#include <cstdint>
#include <string>

namespace rs485_interface
{

/**
 * @brief LC Servo Motor driver (DS2-P/R compatible)
 * 
 * This class implements the driver for LC series servo motors using
 * Modbus RTU protocol over RS485 interface.
 * 
 * Based on DS2-P/R user manual:
 * - Station address: 1 (default)
 * - Baud rate: 19200 (default)
 * - Data format: EVEN-1 (Even parity, 1 stop bit)
 * 
 * Register addresses (calculated from parameter numbers):
 * - P02-00: Control mode (0x0200 = 512)
 * - P0D-17: Force DI enable (0x0D11 = 3345)
 * - P0D-18: Motor enable/disable (0x0D12 = 3346)
 * - P0D-08: Direction control (0x0D08 = 3336)
 * - P10-03: Operating mode (0x1003 = 4099)
 * - P10-42: Speed setting (0x102A = 4138, 32-bit value across 2 registers)
 * - P0B-00: Monitor current speed (0x0B00 = 2816)
 */
class LcServoMotor : public RS485DeviceBaseServo
{
public:
  /**
   * @brief Motor direction options
   */
  enum class Direction : uint16_t
  {
    FORWARD = 16,    // 0x10, 正转
    REVERSE = 32,    // 0x20, 反转
    STOP = 256       // 0x100, 停止
  };

  /**
   * @brief Motor enable state
   */
  enum class EnableState : uint16_t
  {
    DISABLE = 511,   // 0x1FF, 断使能
    ENABLE = 507     // 0x1FB, 电机使能
  };

  /**
   * @brief Constructor
   * @param rs485_client Shared pointer to RS485 client
   * @param slave_address Slave device address (default: 1)
   */
  LcServoMotor(
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
   * @brief Set control mode to Modbus bus mode
   * @return true if successful, false otherwise
   * 
   * Sets P02-00 = 9 (Modbus bus mode)
   */
  bool setModbusMode();


  /**
   * @brief Force DI enable
   * @return true if successful, false otherwise
   * 
   * Sets P0D-17 = 1
   */
  bool forceDIEnable();

  /**
   * @brief Set operating mode to PV mode (speed control)
   * @return true if successful, false otherwise
   * 
   * Sets P10-03 = 3 (PV mode)
   */
  bool setPVMode();

  /**
   * @brief Set motor speed
   * @param speed_prs Speed in Pr/s (Pulse per second)
   * @return true if successful, false otherwise
   * 
   * Sets P10-42 to the specified speed value (32-bit, written as 2 registers)
   * Example: 20000 Pr/s = 120 r/min
   */
  bool setSpeed(uint32_t speed_prs);

  /**
   * @brief Set motor speed in RPM
   * @param speed_rpm Speed in RPM
   * @return true if successful, false otherwise
   * 
   * Converts RPM to Pr/s and sets speed
   * According to manual: 20000 Pr/s = 120 RPM
   * Formula: Pr/s = RPM * (20000 / 120) = RPM * 166.666...
   * This is equivalent to encoder_resolution = 10000 pulses/rev
   */
  bool setSpeedRPM(double speed_rpm);

  /**
   * @brief Set motor speed in RPM with custom encoder resolution
   * @param speed_rpm Speed in RPM
   * @param encoder_resolution Encoder resolution in pulses per revolution
   * @return true if successful, false otherwise
   * 
   * Converts RPM to Pr/s using custom encoder resolution
   * Formula: Pr/s = RPM * (encoder_resolution / 60)
   */
  bool setSpeedRPM(double speed_rpm, double encoder_resolution);

  /**
   * @brief Enable or disable motor
   * @param enable Enable state
   * @return true if successful, false otherwise
   * 
   * Sets P0D-18 (507 = enable, 511 = disable)
   */
  bool setEnable(EnableState enable);

  /**
   * @brief Set motor direction
   * @param direction Direction (FORWARD, REVERSE, or STOP)
   * @return true if successful, false otherwise
   * 
   * Sets P0D-08 (16 = forward, 32 = reverse, 256 = stop)
   */
  bool setDirection(Direction direction);

  /**
   * @brief Read current speed
   * @param speed_rpm Output parameter for current speed in RPM
   * @return true if successful, false otherwise
   * 
   * Reads P0B-00 (current speed in RPM)
   */
  bool readCurrentSpeed(double & speed_rpm);

  /**
   * @brief Read current operating mode
   * @param mode Output parameter for current mode (should be 3 for PV mode)
   * @return true if successful, false otherwise
   * 
   * Reads P10-03 to verify we're in speed control mode
   */
  bool readOperatingMode(uint16_t & mode);

  /**
   * @brief Initialize motor for speed control
   * @return true if successful, false otherwise
   * 
   * Performs complete initialization sequence:
   * 1. Set Modbus mode
   * 2. Force DI enable
   * 3. Set PV mode
   */
  bool initializeSpeedControl();

  /**
   * @brief Read a register value (for debugging)
   * @param register_address Register address to read
   * @param value Output parameter for the read value
   * @return true if successful, false otherwise
   */
  bool readRegister(uint16_t register_address, uint16_t & value);

private:
  // Register addresses (calculated from parameter numbers)
  // Note: According to manual, P02-00 -> 02 00 (hex) -> 0x0200 = 512 (decimal)
  // But some devices use 0-based addressing, so we might need to try 0x01FF (511) as well
  static constexpr uint16_t REG_P02_00 = 0x0200;  // Control mode (512 decimal)
  static constexpr uint16_t REG_P0D_17 = 0x0D11;  // Force DI enable
  static constexpr uint16_t REG_P0D_18 = 0x0D12;  // Motor enable/disable
  static constexpr uint16_t REG_P0D_08 = 0x0D08;  // Direction control
  static constexpr uint16_t REG_P10_03 = 0x1003;  // Operating mode
  static constexpr uint16_t REG_P10_42 = 0x102A;  // Speed setting (32-bit, 2 registers)
  static constexpr uint16_t REG_P0B_00 = 0x0B00;  // Monitor current speed

  // Default encoder resolution (pulses per revolution)
  // Adjust this based on your servo motor specifications
  static constexpr double DEFAULT_ENCODER_RESOLUTION = 10000.0;
};

}  // namespace rs485_interface

#endif  // LC_SERVO_MOTOR_HPP

