#ifndef DEVICE_BASE_HPP
#define DEVICE_BASE_HPP

#include <memory>
#include <string>
#include <cstdint>

namespace rs485_interface
{

class ModbusClient;

/**
 * @brief Base class for RS485 MODBUS devices
 * 
 * This abstract base class defines the interface for all RS485 MODBUS devices.
 * Derived classes should implement device-specific functionality while
 * using the common MODBUS communication provided by ModbusClient.
 */
class DeviceBase
{
public:
  /**
   * @brief Constructor
   * @param modbus_client Shared pointer to MODBUS client
   * @param slave_address MODBUS slave address of the device
   */
  DeviceBase(std::shared_ptr<ModbusClient> modbus_client, uint8_t slave_address);

  /**
   * @brief Virtual destructor
   */
  virtual ~DeviceBase() = default;

  /**
   * @brief Initialize the device
   * @return true if successful, false otherwise
   */
  virtual bool initialize() = 0;

  /**
   * @brief Get device name/type
   * @return Device name string
   */
  virtual std::string getDeviceType() const = 0;

  /**
   * @brief Get last error message
   * @return Error message string
   */
  std::string getLastError() const;

protected:
  std::shared_ptr<ModbusClient> modbus_client_;
  uint8_t slave_address_;
  std::string last_error_;
};

}  // namespace rs485_interface

#endif  // DEVICE_BASE_HPP

