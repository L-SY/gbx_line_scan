#ifndef RS485_CLIENT_HPP
#define RS485_CLIENT_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <mutex>

namespace rs485_interface
{

/**
 * @brief RS485 client for custom protocol communication
 * 
 * This class provides a base implementation for custom RS485 protocol
 * communication over serial interface. It handles serial port configuration,
 * frame construction, CRC16 calculation, and basic read/write operations.
 * 
 * Protocol format is similar to MODBUS RTU but supports custom function codes.
 */
class RS485Client
{
public:
  /**
   * @brief Baud rate options
   */
  enum class BaudRate : int
  {
    BAUD_9600 = 9600,
    BAUD_19200 = 19200,
    BAUD_38400 = 38400,
    BAUD_57600 = 57600,
    BAUD_115200 = 115200,
    BAUD_256000 = 256000
  };

  /**
   * @brief Constructor
   * @param device_path Path to serial device (e.g., /dev/ttyUSB0)
   * @param baud_rate Baud rate for communication
   * @param timeout_ms Read timeout in milliseconds
   */
  RS485Client(
    const std::string & device_path,
    BaudRate baud_rate = BaudRate::BAUD_115200,
    int timeout_ms = 1000
  );

  /**
   * @brief Destructor - closes serial port if open
   */
  ~RS485Client();

  /**
   * @brief Open the serial port
   * @return true if successful, false otherwise
   */
  bool open();

  /**
   * @brief Close the serial port
   */
  void close();

  /**
   * @brief Check if port is open
   * @return true if open, false otherwise
   */
  bool isOpen() const;

  /**
   * @brief Write single register (Function Code 0x06)
   * @param slave_address Slave device address
   * @param register_address Register address to write
   * @param value Value to write (16-bit)
   * @return true if successful, false otherwise
   */
  bool writeSingleRegister(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t value
  );

  /**
   * @brief Write single register without waiting for response (Function Code 0x06)
   * @param slave_address Slave device address
   * @param register_address Register address to write
   * @param value Value to write (16-bit)
   * @return true if successful, false otherwise
   * 
   * This function sends the write command but does not wait for a response.
   * Useful for devices that don't send responses or when response is not needed.
   */
  bool writeSingleRegisterNoResponse(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t value
  );

  /**
   * @brief Write multiple registers (Function Code 0x10)
   * @param slave_address Slave device address
   * @param start_address Starting register address
   * @param values Vector of values to write
   * @return true if successful, false otherwise
   */
  bool writeMultipleRegisters(
    uint8_t slave_address,
    uint16_t start_address,
    const std::vector<uint16_t> & values
  );

  /**
   * @brief Write multiple registers without waiting for response (Function Code 0x10)
   * @param slave_address Slave device address
   * @param start_address Starting register address
   * @param values Vector of values to write
   * @return true if successful, false otherwise
   * 
   * This function sends the write command but does not wait for a response.
   * Useful for devices that don't send responses or when response is not needed.
   */
  bool writeMultipleRegistersNoResponse(
    uint8_t slave_address,
    uint16_t start_address,
    const std::vector<uint16_t> & values
  );

  /**
   * @brief Send raw frame (low-level interface)
   * @param frame Frame data (without CRC, will be added automatically)
   * @return true if successful, false otherwise
   */
  bool sendFrame(const std::vector<uint8_t> & frame);

  /**
   * @brief Receive frame (low-level interface)
   * @param expected_length Expected frame length (without CRC)
   * @param response Received frame data (without CRC verification)
   * @return true if successful, false otherwise
   */
  bool receiveFrame(size_t expected_length, std::vector<uint8_t> & response);

  /**
   * @brief Set timeout for read operations
   * @param timeout_ms Timeout in milliseconds
   */
  void setTimeout(int timeout_ms);

  /**
   * @brief Get last error message
   * @return Error message string
   */
  std::string getLastError() const;

private:
  /**
   * @brief Calculate CRC16 for RS485 communication (MODBUS CRC16)
   * @param data Data buffer
   * @param length Data length
   * @return CRC16 value
   */
  uint16_t calculateCRC16(const uint8_t * data, size_t length) const;

  /**
   * @brief Verify CRC of received frame
   * @param frame Frame data including CRC
   * @return true if CRC is valid
   */
  bool verifyCRC(const std::vector<uint8_t> & frame) const;

  /**
   * @brief Configure serial port
   * @return true if successful
   */
  bool configureSerialPort();

  std::string device_path_;
  BaudRate baud_rate_;
  int timeout_ms_;
  int serial_fd_;
  bool is_open_;
  mutable std::mutex mutex_;
  std::string last_error_;
};

}  // namespace rs485_interface

#endif  // RS485_CLIENT_HPP

