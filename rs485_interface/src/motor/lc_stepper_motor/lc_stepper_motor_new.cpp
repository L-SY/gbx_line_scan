#include "rs485_interface/motor/lc_stepper_motor/lc_stepper_motor_new.hpp"
#include <cstring>

namespace rs485_interface
{

LcStepperMotor::LcStepperMotor(
  std::shared_ptr<RS485Client> rs485_client,
  uint8_t slave_address)
: RS485DeviceBase(rs485_client, slave_address),
  current_position_(0.0f)
{
}

bool LcStepperMotor::initialize()
{
  // Just check if client is ready, don't auto-start
  if (!rs485_client_ || !rs485_client_->isOpen()) {
    setLastError("RS485 client not ready");
    return false;
  }
  return true;
}

std::string LcStepperMotor::getDeviceType() const
{
  return "LC Stepper Motor (CL57)";
}

bool LcStepperMotor::setState(MotorState state)
{
  if (!rs485_client_->writeSingleRegisterNoResponse(
        slave_address_, REG_STATE, static_cast<uint16_t>(state)))
  {
    setLastError("Failed to set state: " + rs485_client_->getLastError());
    return false;
  }
  return true;
}

bool LcStepperMotor::setPosition(float position)
{
  // Position is stored as 32-bit value (2 registers) at address 0x0037
  // Handle negative positions using two's complement
  int32_t pos_int = static_cast<int32_t>(position);
  uint32_t pos;
  if (pos_int >= 0) {
    pos = static_cast<uint32_t>(pos_int);
  } else {
    pos = static_cast<uint32_t>(pos_int);  // Two's complement automatic in C++
  }

  // Convert 32-bit value to two 16-bit registers (high word first)
  std::vector<uint16_t> values;
  values.push_back((pos >> 16) & 0xFFFF);  // High word
  values.push_back(pos & 0xFFFF);           // Low word

  if (!rs485_client_->writeMultipleRegistersNoResponse(
        slave_address_, REG_POSITION, values))
  {
    setLastError("Failed to set position: " + rs485_client_->getLastError());
    return false;
  }
  
  current_position_ = position;
  return true;
}

bool LcStepperMotor::setSpeed(float speed)
{
  uint16_t speed_value = static_cast<uint16_t>(speed);
  
  if (!rs485_client_->writeSingleRegisterNoResponse(
        slave_address_, REG_SPEED, speed_value))
  {
    setLastError("Failed to set speed: " + rs485_client_->getLastError());
    return false;
  }
  return true;
}

bool LcStepperMotor::moveTo(float position, float speed)
{
  // Step 1: Set speed first
  if (!setSpeed(speed)) {
    return false;
  }
  
  // Step 2: Set position (triggers movement)
  if (!setPosition(position)) {
    return false;
  }
  
  return true;
}

bool LcStepperMotor::moveRelative(float distance, float speed)
{
  float new_position = current_position_ + distance;
  return moveTo(new_position, speed);
}

bool LcStepperMotor::getCurrentPosition(float & position)
{
  position = current_position_;
  return true;
}

}  // namespace rs485_interface

