#ifndef DISTANCE_SENSOR_HPP
#define DISTANCE_SENSOR_HPP

#include "rs485_interface/device_base.hpp"
#include "rs485_interface/modbus_client.hpp"

#include <cstdint>
#include <string>

namespace rs485_interface
{

/**
 * @brief SGF Series photoelectric distance sensor driver
 * 
 * This class implements the driver for SGF series photoelectric distance
 * sensors using MODBUS RTU protocol over RS485.
 * 
 * Register addresses based on MODBUS protocol:
 * - Distance reading: 0x0000 (2 registers, value in micrometers)
 * - Working mode: 0x0002
 * - Normally open/closed: 0x0003
 * - Detection output: 0x0004
 * - Analog selection: 0x0005
 * - Differential: 0x0006
 * - External input: 0x0007
 * - Output timing: 0x0008
 * - Display mode: 0x0009
 * - Hold function: 0x000A
 * - Screen off selection: 0x000B
 * - Zeroing value: 0x000C
 * - Threshold 1: 0x000D
 * - Threshold 2: 0x000E
 * - Baud rate: 0x000F
 */
class DistanceSensor : public DeviceBase
{
public:
  /**
   * @brief Working mode options
   */
  enum class WorkingMode : uint16_t
  {
    HIGH_PRECISION = 0,  // 高精度
    STANDARD = 1,        // 标准
    HIGH_SPEED = 2       // 高速
  };

  /**
   * @brief Normally open/closed setting
   */
  enum class SwitchMode : uint16_t
  {
    NORMALLY_OPEN = 0,   // 常开
    NORMALLY_CLOSED = 1  // 常闭
  };

  /**
   * @brief Detection output mode
   */
  enum class DetectionOutput : uint16_t
  {
    NORMAL = 0,          // 正常模式
    ONE_POINT_TEACHING = 1,   // 一点示教
    TWO_POINT_TEACHING = 2,   // 二点示教
    THREE_POINT_TEACHING = 3  // 三点示教
  };

  /**
   * @brief Analog output selection
   */
  enum class AnalogSelection : uint16_t
  {
    VOLTAGE_0_5V = 0,    // 0-5V
    CURRENT_4_20MA = 1   // 4-20mA
  };

  /**
   * @brief Display mode
   */
  enum class DisplayMode : uint16_t
  {
    STANDARD = 0,        // 标准（实际距离）
    REVERSE = 1,         // 反向（0在量程中心）
    OFFSET = 2           // 偏移（0在最远点）
  };

  /**
   * @brief Constructor
   * @param modbus_client Shared pointer to MODBUS client
   * @param slave_address MODBUS slave address (default: 1)
   */
  DistanceSensor(
    std::shared_ptr<ModbusClient> modbus_client,
    uint8_t slave_address = 1
  );

  /**
   * @brief Initialize the sensor
   * @return true if successful, false otherwise
   */
  bool initialize() override;

  /**
   * @brief Get device type
   * @return Device type string
   */
  std::string getDeviceType() const override;

  /**
   * @brief Read distance measurement in millimeters
   * @param distance Distance in millimeters (output)
   * @return true if successful, false otherwise
   */
  bool readDistance(double & distance);

  /**
   * @brief Read distance measurement in micrometers
   * @param distance Distance in micrometers (output)
   * @return true if successful, false otherwise
   */
  bool readDistanceMicrometers(uint32_t & distance);

  /**
   * @brief Read working mode
   * @param mode Working mode (output)
   * @return true if successful, false otherwise
   */
  bool readWorkingMode(WorkingMode & mode);

  /**
   * @brief Set working mode
   * @param mode Working mode to set
   * @return true if successful, false otherwise
   */
  bool setWorkingMode(WorkingMode mode);

  /**
   * @brief Read switch mode (normally open/closed)
   * @param mode Switch mode (output)
   * @return true if successful, false otherwise
   */
  bool readSwitchMode(SwitchMode & mode);

  /**
   * @brief Set switch mode
   * @param mode Switch mode to set
   * @return true if successful, false otherwise
   */
  bool setSwitchMode(SwitchMode mode);

  /**
   * @brief Read detection output mode
   * @param mode Detection output mode (output)
   * @return true if successful, false otherwise
   */
  bool readDetectionOutput(DetectionOutput & mode);

  /**
   * @brief Set detection output mode
   * @param mode Detection output mode to set
   * @return true if successful, false otherwise
   */
  bool setDetectionOutput(DetectionOutput mode);

  /**
   * @brief Read analog selection
   * @param selection Analog selection (output)
   * @return true if successful, false otherwise
   */
  bool readAnalogSelection(AnalogSelection & selection);

  /**
   * @brief Set analog selection
   * @param selection Analog selection to set
   * @return true if successful, false otherwise
   */
  bool setAnalogSelection(AnalogSelection selection);

  /**
   * @brief Read differential (switch-off distance)
   * @param differential Differential value (output)
   * @return true if successful, false otherwise
   */
  bool readDifferential(uint16_t & differential);

  /**
   * @brief Set differential
   * @param differential Differential value to set
   * @return true if successful, false otherwise
   */
  bool setDifferential(uint16_t differential);

  /**
   * @brief Read display mode
   * @param mode Display mode (output)
   * @return true if successful, false otherwise
   */
  bool readDisplayMode(DisplayMode & mode);

  /**
   * @brief Set display mode
   * @param mode Display mode to set
   * @return true if successful, false otherwise
   */
  bool setDisplayMode(DisplayMode mode);

  /**
   * @brief Read baud rate
   * @param baud_rate Baud rate index (0-5: 9600, 19200, 38400, 57600, 115200, 256000)
   * @return true if successful, false otherwise
   */
  bool readBaudRate(uint16_t & baud_rate);

  /**
   * @brief Set baud rate
   * @param baud_rate_index Baud rate index (0-5)
   * @return true if successful, false otherwise
   */
  bool setBaudRate(uint16_t baud_rate_index);

  /**
   * @brief Perform zeroing operation
   * @return true if successful, false otherwise
   */
  bool zero();

  /**
   * @brief Perform teaching operation
   * @return true if successful, false otherwise
   */
  bool teach();

  /**
   * @brief Stop laser measurement
   * @return true if successful, false otherwise
   */
  bool stopLaser();

  /**
   * @brief Reset to factory settings
   * @return true if successful, false otherwise
   */
  bool reset();

private:
  static constexpr uint16_t REG_DISTANCE = 0x0000;      // Distance register (2 registers)
  static constexpr uint16_t REG_WORKING_MODE = 0x0002;
  static constexpr uint16_t REG_SWITCH_MODE = 0x0003;
  static constexpr uint16_t REG_DETECTION_OUTPUT = 0x0004;
  static constexpr uint16_t REG_ANALOG_SELECTION = 0x0005;
  static constexpr uint16_t REG_DIFFERENTIAL = 0x0006;
  static constexpr uint16_t REG_EXTERNAL_INPUT = 0x0007;
  static constexpr uint16_t REG_OUTPUT_TIMING = 0x0008;
  static constexpr uint16_t REG_DISPLAY_MODE = 0x0009;
  static constexpr uint16_t REG_HOLD = 0x000A;
  static constexpr uint16_t REG_SCREEN_OFF = 0x000B;
  static constexpr uint16_t REG_ZERO_VALUE = 0x000C;
  static constexpr uint16_t REG_THRESHOLD_1 = 0x000D;
  static constexpr uint16_t REG_THRESHOLD_2 = 0x000E;
  static constexpr uint16_t REG_BAUD_RATE = 0x000F;

  // Write-only operation registers
  static constexpr uint16_t REG_OPERATION_CONTINUOUS = 0x0001;  // 连续/断续输出
  static constexpr uint16_t REG_OPERATION_ZERO = 0x0010;        // 调零
  static constexpr uint16_t REG_OPERATION_TEACH = 0x0011;       // 示教
  static constexpr uint16_t REG_OPERATION_STOP = 0x0012;        // 停止激光
  static constexpr uint16_t REG_OPERATION_RESET = 0x0013;       // 复位
};

}  // namespace rs485_interface

#endif  // DISTANCE_SENSOR_HPP

