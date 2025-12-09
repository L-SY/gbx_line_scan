#include "rs485_interface/device_base.hpp"
#include "rs485_interface/modbus_client.hpp"

namespace rs485_interface
{

DeviceBase::DeviceBase(
  std::shared_ptr<ModbusClient> modbus_client,
  uint8_t slave_address)
: modbus_client_(modbus_client),
  slave_address_(slave_address)
{
}

std::string DeviceBase::getLastError() const
{
  if (!last_error_.empty()) {
    return last_error_;
  }
  if (modbus_client_) {
    return modbus_client_->getLastError();
  }
  return "No error";
}

}  // namespace rs485_interface

