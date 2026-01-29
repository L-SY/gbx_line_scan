#include "rs485_interface/motor/lc_servo_motor/lc_servo_motor_new.hpp"
#include <thread>
#include <chrono>

namespace rs485_interface
{

LcServoMotor::LcServoMotor(
  std::shared_ptr<RS485Client> rs485_client,
  uint8_t slave_address)
: RS485DeviceBase(rs485_client, slave_address)
{
}

bool LcServoMotor::initialize()
{
  return initializeSpeedControl();
}

std::string LcServoMotor::getDeviceType() const
{
  return "LC Servo Motor";
}

bool LcServoMotor::setModbusMode()
{
  return rs485_client_->writeSingleRegister(slave_address_, REG_P02_00, 9);
}

bool LcServoMotor::forceDIEnable()
{
  return rs485_client_->writeSingleRegister(slave_address_, REG_P0D_17, 1);
}

bool LcServoMotor::setPVMode()
{
  return rs485_client_->writeSingleRegister(slave_address_, REG_P10_03, 3);
}

bool LcServoMotor::setSpeed(uint32_t speed_prs)
{
  std::vector<uint16_t> values;
  values.push_back(static_cast<uint16_t>(speed_prs & 0xFFFF));
  values.push_back(static_cast<uint16_t>((speed_prs >> 16) & 0xFFFF));
  return rs485_client_->writeMultipleRegisters(slave_address_, REG_P10_42, values);
}

bool LcServoMotor::setSpeedRPM(double speed_rpm)
{
  return setSpeedRPM(speed_rpm, DEFAULT_ENCODER_RESOLUTION);
}

bool LcServoMotor::setSpeedRPM(double speed_rpm, double encoder_resolution)
{
  double speed_prs = speed_rpm * (encoder_resolution / 60.0);
  return setSpeed(static_cast<uint32_t>(speed_prs));
}

bool LcServoMotor::setEnable(EnableState enable)
{
  return rs485_client_->writeSingleRegister(slave_address_, REG_P0D_18, static_cast<uint16_t>(enable));
}

bool LcServoMotor::setDirection(Direction direction)
{
  return rs485_client_->writeSingleRegister(slave_address_, REG_P0D_08, static_cast<uint16_t>(direction));
}

bool LcServoMotor::readCurrentSpeed(double & speed_rpm)
{
  std::vector<uint16_t> result;
  if (!rs485_client_->readHoldingRegisters(slave_address_, REG_P0B_00, 1, result)) {
    return false;
  }
  if (result.empty()) {
    setLastError("Empty response");
    return false;
  }
  speed_rpm = static_cast<double>(static_cast<int16_t>(result[0]));
  return true;
}

bool LcServoMotor::readOperatingMode(uint16_t & mode)
{
  std::vector<uint16_t> result;
  if (!rs485_client_->readHoldingRegisters(slave_address_, REG_P10_03, 1, result)) {
    return false;
  }
  if (result.empty()) {
    setLastError("Empty response");
    return false;
  }
  mode = result[0];
  return true;
}

bool LcServoMotor::setModbusDataFormat(uint16_t format)
{
  // POC-03: Modbus data format
  // 0: No parity, 2 stop bits
  // 1: Even parity, 1 stop bit (factory default)
  // 2: Odd parity, 1 stop bit
  // 3: No parity, 1 stop bit (for NONE parity with 1 stop bit)
  
  // Try to set the format
  if (rs485_client_->writeSingleRegister(slave_address_, REG_POC_03, format)) {
    // Wait for device to process
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
  }
  
  // If failed, it might be because the current parity doesn't match
  // (e.g., motor is in EVEN parity mode but we're using NONE parity)
  // Try to read the current value first to see if communication works
  uint16_t current_value = 0;
  if (readRegister(REG_POC_03, current_value)) {
    // Communication works, but write failed
    if (current_value == format) {
      // Already set to the desired format
      return true;
    }
    setLastError("Failed to set Modbus data format: " + rs485_client_->getLastError());
    return false;
  }
  
  // Can't even read, parity mismatch likely
  // Return false but don't set error, let caller handle it
  setLastError("Failed to communicate with motor (parity mismatch?): " + rs485_client_->getLastError());
  return false;
}

bool LcServoMotor::initializeSpeedControl()
{
  // First, try to set Modbus data format to NONE parity (format 3)
  // This allows the motor to work with NONE parity instead of EVEN
  // If this fails (e.g., motor is still in EVEN parity mode), we'll continue anyway
  // The motor might already be configured correctly, or we'll need to configure it manually
  bool format_set = setModbusDataFormat(3);
  if (!format_set) {
    // If setting format failed, it's likely because motor is in EVEN parity mode
    // but we're using NONE parity. Continue anyway - the motor might work
    // or user needs to configure POC-03 manually first
  }
  
  // Continue with initialization even if format setting failed
  if (!setModbusMode()) {
    setLastError("Failed to set Modbus mode");
    return false;
  }
  if (!forceDIEnable()) {
    setLastError("Failed to force DI enable");
    return false;
  }
  if (!setPVMode()) {
    setLastError("Failed to set PV mode");
    return false;
  }
  return true;
}

bool LcServoMotor::readRegister(uint16_t register_address, uint16_t & value)
{
  std::vector<uint16_t> result;
  if (!rs485_client_->readHoldingRegisters(slave_address_, register_address, 1, result)) {
    return false;
  }
  if (result.empty()) {
    setLastError("Empty response");
    return false;
  }
  value = result[0];
  return true;
}

}  // namespace rs485_interface


