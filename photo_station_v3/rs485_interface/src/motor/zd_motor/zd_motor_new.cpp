#include "rs485_interface/motor/zd_motor/zd_motor_new.hpp"

namespace rs485_interface
{

ZdMotor::ZdMotor(
  std::shared_ptr<RS485Client> rs485_client,
  uint8_t slave_address)
: RS485DeviceBase(rs485_client, slave_address)
{
}

bool ZdMotor::initialize()
{
  // Try to read status to verify communication
  uint16_t status;
  if (!readStatusWord1(status)) {
    setLastError("Failed to communicate with ZD Motor");
    return false;
  }
  return true;
}

std::string ZdMotor::getDeviceType() const
{
  return "ZD Motor";
}

bool ZdMotor::scanMotorId(
  std::shared_ptr<RS485Client> rs485_client,
  uint8_t & found_id,
  uint8_t start_id,
  uint8_t end_id)
{
  for (uint8_t id = start_id; id <= end_id; ++id) {
    std::vector<uint16_t> result;
    if (rs485_client->readHoldingRegisters(id, REG_SLAVE_ADDRESS, 1, result)) {
      if (!result.empty()) {
        found_id = id;
        return true;
      }
    }
  }
  return false;
}

bool ZdMotor::readMotorId(uint8_t & id)
{
  std::vector<uint16_t> result;
  if (!rs485_client_->readHoldingRegisters(slave_address_, REG_SLAVE_ADDRESS, 1, result)) {
    return false;
  }
  if (result.empty()) {
    setLastError("Empty response");
    return false;
  }
  id = static_cast<uint8_t>(result[0] & 0xFF);
  return true;
}

bool ZdMotor::setMotorId(uint8_t new_id)
{
  // Enable write to F group registers
  if (!rs485_client_->writeSingleRegister(slave_address_, REG_WRITE_ENABLE, 0x01)) {
    return false;
  }
  // Set new ID
  return rs485_client_->writeSingleRegister(slave_address_, REG_SLAVE_ADDRESS, new_id);
}

bool ZdMotor::readRegister(uint16_t register_address, uint16_t & value)
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

bool ZdMotor::writeRegister(uint16_t register_address, uint16_t value)
{
  return rs485_client_->writeSingleRegister(slave_address_, register_address, value);
}

bool ZdMotor::setControlCommand(ControlCommand command)
{
  return writeRegister(REG_CONTROL_CMD, static_cast<uint16_t>(command));
}

bool ZdMotor::setSpeedRPM(uint16_t speed_rpm)
{
  return writeRegister(REG_SPEED_SETTING, speed_rpm);
}

bool ZdMotor::readStatusWord1(uint16_t & status)
{
  return readRegister(REG_STATUS_WORD1, status);
}

bool ZdMotor::readStatusWord2(uint16_t & status)
{
  return readRegister(REG_STATUS_WORD2, status);
}

}  // namespace rs485_interface


