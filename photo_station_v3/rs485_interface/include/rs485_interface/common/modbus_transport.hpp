#ifndef RS485_INTERFACE__COMMON__MODBUS_TRANSPORT_HPP
#define RS485_INTERFACE__COMMON__MODBUS_TRANSPORT_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rs485_interface
{

/**
 * @brief Minimal Modbus RTU transport interface used by DeviceBase/DistanceSensor.
 *
 * Note: this interface intentionally matches the subset of ModbusClient API
 * that existing devices use, so transports can be backed by different serial clients.
 */
class ModbusTransport
{
public:
  virtual ~ModbusTransport() = default;

  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;

  // Compatibility with original ModbusClient naming.
  // In the legacy ModbusClient implementation this uses function code 0x04.
  virtual bool readHoldingRegisters(
    uint8_t slave_address,
    uint16_t start_address,
    uint16_t num_registers,
    std::vector<uint16_t> & result) = 0;

  virtual bool writeSingleRegister(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t value) = 0;

  virtual bool writeMultipleRegisters(
    uint8_t slave_address,
    uint16_t start_address,
    const std::vector<uint16_t> & values) = 0;

  virtual void setTimeout(int timeout_ms) = 0;
  virtual std::string getLastError() const = 0;
};

}  // namespace rs485_interface

#endif  // RS485_INTERFACE__COMMON__MODBUS_TRANSPORT_HPP

