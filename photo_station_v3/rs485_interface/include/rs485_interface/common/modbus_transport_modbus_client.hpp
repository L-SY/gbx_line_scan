#ifndef RS485_INTERFACE__COMMON__MODBUS_TRANSPORT_MODBUS_CLIENT_HPP
#define RS485_INTERFACE__COMMON__MODBUS_TRANSPORT_MODBUS_CLIENT_HPP

#include "rs485_interface/common/modbus_transport.hpp"
#include "rs485_interface/distance_sensor/modbus_client.hpp"

#include <memory>
#include <string>
#include <vector>

namespace rs485_interface
{

/**
 * @brief Adapter: ModbusTransport backed by legacy ModbusClient.
 */
class ModbusClientTransport final : public ModbusTransport
{
public:
  explicit ModbusClientTransport(std::shared_ptr<ModbusClient> client)
  : client_(std::move(client)) {}

  bool open() override { return client_ ? client_->open() : false; }
  void close() override { if (client_) { client_->close(); } }
  bool isOpen() const override { return client_ && client_->isOpen(); }

  bool readHoldingRegisters(
    uint8_t slave_address,
    uint16_t start_address,
    uint16_t num_registers,
    std::vector<uint16_t> & result) override
  {
    return client_ && client_->readHoldingRegisters(slave_address, start_address, num_registers, result);
  }

  bool writeSingleRegister(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t value) override
  {
    return client_ && client_->writeSingleRegister(slave_address, register_address, value);
  }

  bool writeMultipleRegisters(
    uint8_t slave_address,
    uint16_t start_address,
    const std::vector<uint16_t> & values) override
  {
    return client_ && client_->writeMultipleRegisters(slave_address, start_address, values);
  }

  void setTimeout(int timeout_ms) override { if (client_) { client_->setTimeout(timeout_ms); } }
  std::string getLastError() const override { return client_ ? client_->getLastError() : "ModbusClient is null"; }

private:
  std::shared_ptr<ModbusClient> client_;
};

}  // namespace rs485_interface

#endif  // RS485_INTERFACE__COMMON__MODBUS_TRANSPORT_MODBUS_CLIENT_HPP

