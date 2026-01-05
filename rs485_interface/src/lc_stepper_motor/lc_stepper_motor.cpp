#include "rs485_interface/lc_stepper_motor/lc_stepper_motor.hpp"
#include <cstring>

namespace rs485_interface
{

LcStepperMotor::LcStepperMotor(
  std::shared_ptr<RS485Client> rs485_client,
  uint8_t slave_address)
: RS485DeviceBase(rs485_client, slave_address)
{
}

bool LcStepperMotor::initialize()
{
  if (!rs485_client_) {
    last_error_ = "RS485 client is null";
    return false;
  }

  if (!rs485_client_->isOpen()) {
    last_error_ = "RS485 client is not open";
    return false;
  }

  // Motor is ready, but state needs to be set explicitly by user
  last_error_.clear();
  return true;
}

std::string LcStepperMotor::getDeviceType() const
{
  return "LC Stepper Motor (CL57 compatible)";
}

bool LcStepperMotor::setState(MotorState state)
{
  // CL57 motor doesn't send responses, use NoResponse version
  if (!rs485_client_->writeSingleRegisterNoResponse(
        slave_address_,
        REG_STATE,
        static_cast<uint16_t>(state)))
  {
    last_error_ = "Failed to set motor state: " + rs485_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

bool LcStepperMotor::setPosition(float position)
{
  // CL57 uses Function Code 0x10 (write multiple registers) for position
  // Position is stored as 32-bit value (2 registers) at address 0x0037
  // Handle negative positions using two's complement
  uint32_t pos;
  if (position >= 0) {
    pos = static_cast<uint32_t>(position);
  } else {
    pos = 0xFFFFFFFF + static_cast<int32_t>(position);
  }

  // Convert 32-bit value to two 16-bit registers (high word first)
  std::vector<uint16_t> values;
  values.push_back((pos >> 16) & 0xFFFF);  // High word
  values.push_back(pos & 0xFFFF);           // Low word

  // CL57 motor doesn't send responses, use NoResponse version
  if (!rs485_client_->writeMultipleRegistersNoResponse(
        slave_address_,
        REG_POSITION,
        values))
  {
    last_error_ = "Failed to set motor position: " + rs485_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

bool LcStepperMotor::setSpeed(float speed)
{
  // CL57 uses Function Code 0x06 (write single register) for speed
  // Speed is in RPM, convert float to uint16_t
  uint16_t speed_value = static_cast<uint16_t>(speed);

  // CL57 motor doesn't send responses, use NoResponse version
  if (!rs485_client_->writeSingleRegisterNoResponse(
        slave_address_,
        REG_SPEED,
        speed_value))
  {
    last_error_ = "Failed to set motor speed: " + rs485_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

}  // namespace rs485_interface

