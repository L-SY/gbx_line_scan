#include "rs485_interface/motor/lc_servo_motor/lc_servo_motor_new.hpp"

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

bool LcServoMotor::initializeSpeedControl()
{
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


