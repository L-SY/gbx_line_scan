#include "rs485_interface/motor/lc_servo_motor/rs485_device_base.hpp"
#include "rs485_interface/motor/lc_servo_motor/rs485_client.hpp"

namespace rs485_interface
{

RS485DeviceBaseServo::RS485DeviceBaseServo(
  std::shared_ptr<RS485ClientServo> rs485_client,
  uint8_t slave_address)
: rs485_client_(rs485_client),
  slave_address_(slave_address)
{
}

std::string RS485DeviceBaseServo::getLastError() const
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

