#ifndef RS485_INTERFACE__COMMON__MODBUS_TRANSPORT_RS485_CLIENT_HPP
#define RS485_INTERFACE__COMMON__MODBUS_TRANSPORT_RS485_CLIENT_HPP

#include "rs485_interface/common/modbus_transport.hpp"
#include "rs485_interface/common/rs485_client.hpp"

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace rs485_interface
{

/**
 * @brief ModbusTransport backed by the unified RS485Client.
 *
 * Important: legacy ModbusClient::readHoldingRegisters() uses function code 0x04.
 * RS485Client::readHoldingRegisters() uses function code 0x03.
 * For SGF distance sensor we keep using 0x04 to match existing driver behavior.
 */
class Rs485ClientTransport final : public ModbusTransport
{
public:
  explicit Rs485ClientTransport(std::shared_ptr<RS485Client> client)
  : client_(std::move(client)) {}

  bool open() override
  {
    // We assume the shared RS485Client is managed by the caller (already opened).
    if (!client_) {
      last_error_ = "RS485Client is null";
      return false;
    }
    return client_->isOpen();
  }

  void close() override
  {
    // No-op: shared client lifetime managed externally.
  }

  bool isOpen() const override
  {
    return client_ && client_->isOpen();
  }

  bool readHoldingRegisters(
    uint8_t slave_address,
    uint16_t start_address,
    uint16_t num_registers,
    std::vector<uint16_t> & result) override
  {
    if (!client_) {
      last_error_ = "RS485Client is null";
      return false;
    }
    if (!client_->isOpen()) {
      last_error_ = "Port not open";
      return false;
    }
    if (num_registers == 0 || num_registers > 125) {
      last_error_ = "Invalid number of registers (1-125)";
      return false;
    }

    // Build request frame: [SlaveAddr][0x04][StartAddrHi][StartAddrLo][NumRegHi][NumRegLo]
    std::vector<uint8_t> frame;
    frame.push_back(slave_address);
    frame.push_back(0x04);  // Function Code: Read Input Registers (legacy ModbusClient behavior)
    frame.push_back((start_address >> 8) & 0xFF);
    frame.push_back(start_address & 0xFF);
    frame.push_back((num_registers >> 8) & 0xFF);
    frame.push_back(num_registers & 0xFF);

    if (!client_->sendFrame(frame)) {
      last_error_ = client_->getLastError();
      return false;
    }

    // Expected response data length (without CRC): [SlaveAddr][0x04][ByteCount][Data...]
    size_t expected_length = 3 + num_registers * 2;
    std::vector<uint8_t> response;
    if (!client_->receiveFrame(expected_length, response)) {
      last_error_ = client_->getLastError();
      return false;
    }

    // Exception response: [SlaveAddr][0x84][ExceptionCode]
    if (response.size() == 3 && response[0] == slave_address && response[1] == 0x84) {
      uint8_t error_code = response[2];
      char hex[3];
      std::snprintf(hex, sizeof(hex), "%02X", error_code);
      last_error_ = std::string("Modbus exception response: error code 0x") + hex;
      return false;
    }

    if (response.size() < 3) {
      last_error_ = "Invalid response length";
      return false;
    }
    if (response[0] != slave_address || response[1] != 0x04) {
      last_error_ = "Response does not match request";
      return false;
    }

    uint8_t byte_count = response[2];
    if (byte_count != num_registers * 2) {
      last_error_ = "Byte count mismatch";
      return false;
    }

    result.clear();
    result.reserve(num_registers);
    for (size_t i = 0; i < num_registers; ++i) {
      uint16_t value = (static_cast<uint16_t>(response[3 + i * 2]) << 8) |
                       static_cast<uint16_t>(response[3 + i * 2 + 1]);
      result.push_back(value);
    }

    last_error_.clear();
    return true;
  }

  bool writeSingleRegister(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t value) override
  {
    if (!client_) {
      last_error_ = "RS485Client is null";
      return false;
    }
    if (!client_->writeSingleRegister(slave_address, register_address, value)) {
      last_error_ = client_->getLastError();
      return false;
    }
    last_error_.clear();
    return true;
  }

  bool writeMultipleRegisters(
    uint8_t slave_address,
    uint16_t start_address,
    const std::vector<uint16_t> & values) override
  {
    if (!client_) {
      last_error_ = "RS485Client is null";
      return false;
    }
    if (!client_->writeMultipleRegisters(slave_address, start_address, values)) {
      last_error_ = client_->getLastError();
      return false;
    }
    last_error_.clear();
    return true;
  }

  void setTimeout(int timeout_ms) override
  {
    if (client_) {
      client_->setTimeout(timeout_ms);
    }
  }

  std::string getLastError() const override
  {
    if (!last_error_.empty()) {
      return last_error_;
    }
    return client_ ? client_->getLastError() : "RS485Client is null";
  }

private:
  std::shared_ptr<RS485Client> client_;
  std::string last_error_;
};

}  // namespace rs485_interface

#endif  // RS485_INTERFACE__COMMON__MODBUS_TRANSPORT_RS485_CLIENT_HPP

