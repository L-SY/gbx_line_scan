#include "rs485_interface/lc_servo_motor/lc_servo_motor.hpp"
#include <cstring>

namespace rs485_interface
{

LcServoMotor::LcServoMotor(
  std::shared_ptr<RS485ClientServo> rs485_client,
  uint8_t slave_address)
: RS485DeviceBaseServo(rs485_client, slave_address)
{
}

bool LcServoMotor::initialize()
{
  if (!rs485_client_) {
    last_error_ = "RS485 client is null";
    return false;
  }

  if (!rs485_client_->isOpen()) {
    last_error_ = "RS485 client is not open";
    return false;
  }

  // Motor is ready, but initialization sequence needs to be called explicitly
  last_error_.clear();
  return true;
}

std::string LcServoMotor::getDeviceType() const
{
  return "LC Servo Motor (DS2-P/R compatible)";
}

bool LcServoMotor::setModbusMode()
{
  // P02-00 = 9 (Modbus bus mode)
  if (!rs485_client_->writeSingleRegister(slave_address_, REG_P02_00, 9)) {
    last_error_ = "Failed to set Modbus mode: " + rs485_client_->getLastError();
    return false;
  }
  last_error_.clear();
  return true;
}

bool LcServoMotor::forceDIEnable()
{
  // P0D-17 = 1 (Force DI enable)
  if (!rs485_client_->writeSingleRegister(slave_address_, REG_P0D_17, 1)) {
    last_error_ = "Failed to force DI enable: " + rs485_client_->getLastError();
    return false;
  }
  last_error_.clear();
  return true;
}

bool LcServoMotor::setPVMode()
{
  // P10-03 = 3 (PV mode - speed control)
  if (!rs485_client_->writeSingleRegister(slave_address_, REG_P10_03, 3)) {
    last_error_ = "Failed to set PV mode: " + rs485_client_->getLastError();
    return false;
  }
  last_error_.clear();
  return true;
}

bool LcServoMotor::setSpeed(uint32_t speed_prs)
{
  // P10-42 is a 32-bit value stored in 2 registers (high word first, low word second)
  // Address 0x102A is the high 16-bit, 0x102B is the low 16-bit
  uint16_t high_word = (speed_prs >> 16) & 0xFFFF;
  uint16_t low_word = speed_prs & 0xFFFF;

  std::vector<uint16_t> values;
  values.push_back(high_word);
  values.push_back(low_word);

  if (!rs485_client_->writeMultipleRegisters(slave_address_, REG_P10_42, values)) {
    last_error_ = "Failed to set speed: " + rs485_client_->getLastError();
    return false;
  }
  last_error_.clear();
  return true;
}

bool LcServoMotor::setSpeedRPM(double speed_rpm)
{
  // Convert RPM to Pr/s
  // Formula: Pr/s = RPM * (encoder_resolution / 60)
  double speed_prs = speed_rpm * (DEFAULT_ENCODER_RESOLUTION / 60.0);
  uint32_t speed_prs_int = static_cast<uint32_t>(speed_prs);
  return setSpeed(speed_prs_int);
}

bool LcServoMotor::setEnable(EnableState enable)
{
  // P0D-18: 507 = enable, 511 = disable
  uint16_t value = static_cast<uint16_t>(enable);
  if (!rs485_client_->writeSingleRegister(slave_address_, REG_P0D_18, value)) {
    last_error_ = "Failed to set enable state: " + rs485_client_->getLastError();
    return false;
  }
  last_error_.clear();
  return true;
}

bool LcServoMotor::setDirection(Direction direction)
{
  // P0D-08: 16 = forward, 32 = reverse, 256 = stop
  uint16_t value = static_cast<uint16_t>(direction);
  if (!rs485_client_->writeSingleRegister(slave_address_, REG_P0D_08, value)) {
    last_error_ = "Failed to set direction: " + rs485_client_->getLastError();
    return false;
  }
  last_error_.clear();
  return true;
}

bool LcServoMotor::readCurrentSpeed(double & speed_rpm)
{
  // Read P0B-00 (current speed in RPM)
  std::vector<uint16_t> result;
  if (!rs485_client_->readHoldingRegisters(slave_address_, REG_P0B_00, 1, result)) {
    last_error_ = "Failed to read current speed: " + rs485_client_->getLastError();
    return false;
  }

  if (result.empty()) {
    last_error_ = "No data received";
    return false;
  }

  speed_rpm = static_cast<double>(result[0]);
  last_error_.clear();
  return true;
}

bool LcServoMotor::initializeSpeedControl()
{
  // Complete initialization sequence for speed control
  if (!setModbusMode()) {
    return false;
  }

  if (!forceDIEnable()) {
    return false;
  }

  if (!setPVMode()) {
    return false;
  }

  last_error_.clear();
  return true;
}

}  // namespace rs485_interface

