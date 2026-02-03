#include "rs485_interface/common/device_base.hpp"
#include "rs485_interface/common/modbus_transport.hpp"

namespace rs485_interface
{

DeviceBase::DeviceBase(
  std::shared_ptr<ModbusTransport> modbus,
  uint8_t slave_address)
: modbus_(modbus),
  slave_address_(slave_address)
{
}

std::string DeviceBase::getLastError() const
{
  if (!last_error_.empty()) {
    return last_error_;
  }
  if (modbus_) {
    return modbus_->getLastError();
  }
  return "No error";
}

}  // namespace rs485_interface

