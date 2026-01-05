#ifndef RS485_DEVICE_BASE_HPP
#define RS485_DEVICE_BASE_HPP

#include <memory>
#include <string>
#include <cstdint>

namespace rs485_interface
{

class RS485Client;

/**
 * @brief Base class for RS485 devices using custom protocol
 * 
 * This abstract base class defines the interface for all RS485 devices
 * using custom protocol communication. Derived classes should implement
 * device-specific functionality while using the common RS485 communication
 * provided by RS485Client.
 */
class RS485DeviceBase
{
public:
  /**
   * @brief Constructor
   * @param rs485_client Shared pointer to RS485 client
   * @param slave_address Slave device address
   */
  RS485DeviceBase(std::shared_ptr<RS485Client> rs485_client, uint8_t slave_address);

  /**
   * @brief Virtual destructor
   */
  virtual ~RS485DeviceBase() = default;

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
  std::shared_ptr<RS485Client> rs485_client_;
  uint8_t slave_address_;
  std::string last_error_;
};

}  // namespace rs485_interface

#endif  // RS485_DEVICE_BASE_HPP

