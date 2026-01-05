#include "rs485_interface/lc_stepper_motor/rs485_device_base.hpp"
#include "rs485_interface/lc_stepper_motor/rs485_client.hpp"

namespace rs485_interface
{

RS485DeviceBase::RS485DeviceBase(
  std::shared_ptr<RS485Client> rs485_client,
  uint8_t slave_address)
: rs485_client_(rs485_client),
  slave_address_(slave_address)
{
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

}  // namespace rs485_interface

