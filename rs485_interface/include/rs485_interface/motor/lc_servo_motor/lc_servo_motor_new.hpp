#ifndef LC_SERVO_MOTOR_NEW_HPP
#define LC_SERVO_MOTOR_NEW_HPP

#include "rs485_interface/common/rs485_device_base.hpp"
#include "rs485_interface/common/rs485_client.hpp"

#include <cstdint>
#include <string>

namespace rs485_interface
{

/**
 * @brief LC Servo Motor driver (DS2-P/R compatible) - Refactored
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
class LcServoMotor : public RS485DeviceBase
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
    std::shared_ptr<RS485Client> rs485_client,
    uint8_t slave_address = 1
  );

  /**
   * @brief Initialize the motor
   * @return true if successful, false otherwise
   */
  bool initialize() override;

  /**
   * @brief Get device type string
   * @return Device type string
   */
  std::string getDeviceType() const override;

  /**
   * @brief Get motor type enum
   * @return Motor type
   */
  MotorType getMotorType() const override { return MotorType::LC_SERVO_MOTOR; }

  /**
   * @brief Set control mode to Modbus bus mode
   * @return true if successful, false otherwise
   */
  bool setModbusMode();

  /**
   * @brief Force DI enable
   * @return true if successful, false otherwise
   */
  bool forceDIEnable();

  /**
   * @brief Set operating mode to PV mode (speed control)
   * @return true if successful, false otherwise
   */
  bool setPVMode();

  /**
   * @brief Set motor speed
   * @param speed_prs Speed in Pr/s (Pulse per second)
   * @return true if successful, false otherwise
   */
  bool setSpeed(uint32_t speed_prs);

  /**
   * @brief Set motor speed in RPM
   * @param speed_rpm Speed in RPM
   * @return true if successful, false otherwise
   */
  bool setSpeedRPM(double speed_rpm);

  /**
   * @brief Set motor speed in RPM with custom encoder resolution
   * @param speed_rpm Speed in RPM
   * @param encoder_resolution Encoder resolution in pulses per revolution
   * @return true if successful, false otherwise
   */
  bool setSpeedRPM(double speed_rpm, double encoder_resolution);

  /**
   * @brief Enable or disable motor
   * @param enable Enable state
   * @return true if successful, false otherwise
   */
  bool setEnable(EnableState enable);

  /**
   * @brief Set motor direction
   * @param direction Direction (FORWARD, REVERSE, or STOP)
   * @return true if successful, false otherwise
   */
  bool setDirection(Direction direction);

  /**
   * @brief Read current speed
   * @param speed_rpm Output parameter for current speed in RPM
   * @return true if successful, false otherwise
   */
  bool readCurrentSpeed(double & speed_rpm);

  /**
   * @brief Read current operating mode
   * @param mode Output parameter for current mode
   * @return true if successful, false otherwise
   */
  bool readOperatingMode(uint16_t & mode);

  /**
   * @brief Initialize motor for speed control
   * @return true if successful, false otherwise
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
  static constexpr uint16_t REG_P02_00 = 0x0200;
  static constexpr uint16_t REG_P0D_17 = 0x0D11;
  static constexpr uint16_t REG_P0D_18 = 0x0D12;
  static constexpr uint16_t REG_P0D_08 = 0x0D08;
  static constexpr uint16_t REG_P10_03 = 0x1003;
  static constexpr uint16_t REG_P10_42 = 0x102A;
  static constexpr uint16_t REG_P0B_00 = 0x0B00;

  static constexpr double DEFAULT_ENCODER_RESOLUTION = 10000.0;
};

}  // namespace rs485_interface

#endif  // LC_SERVO_MOTOR_NEW_HPP

