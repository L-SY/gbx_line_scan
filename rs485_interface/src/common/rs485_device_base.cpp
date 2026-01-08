#include "rs485_interface/common/rs485_device_base.hpp"
#include "rs485_interface/common/rs485_client.hpp"

namespace rs485_interface
{

RS485DeviceBase::RS485DeviceBase(
  std::shared_ptr<RS485Client> rs485_client,
  uint8_t slave_address)
: rs485_client_(rs485_client),
  slave_address_(slave_address)
{
}

uint8_t RS485DeviceBase::getSlaveAddress() const
{
  return slave_address_;
}

void RS485DeviceBase::setSlaveAddress(uint8_t address)
{
  slave_address_ = address;
}

std::shared_ptr<RS485Client> RS485DeviceBase::getRS485Client() const
{
  return rs485_client_;
}

bool RS485DeviceBase::isConnected() const
{
  return rs485_client_ && rs485_client_->isOpen();
}

std::string RS485DeviceBase::getLastError() const
{
  if (!last_error_.empty()) {
    return last_error_;
  }
  if (rs485_client_) {
    return rs485_client_->getLastError();
  }
  return "No error";
}

void RS485DeviceBase::setLastError(const std::string & error)
{
  last_error_ = error;
}

}  // namespace rs485_interface

