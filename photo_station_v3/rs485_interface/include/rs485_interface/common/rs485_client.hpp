#ifndef RS485_COMMON_CLIENT_HPP
#define RS485_COMMON_CLIENT_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <mutex>

namespace rs485_interface
{

/**
 * @brief Unified RS485 client for Modbus RTU protocol communication
 * 
 * This class provides a unified implementation for RS485 serial communication
 * supporting various configurations for different motor types:
 * - Configurable baud rates (2400 - 256000)
 * - Configurable parity (NONE, EVEN, ODD)
 * - Standard Modbus function codes (0x03, 0x06, 0x10)
 * - Optional NoResponse variants for devices that don't send responses
 * 
 * This unified client replaces the separate RS485ClientServo and RS485Client
 * classes used by different motor types.
 */
class RS485Client
{
public:
  /**
   * @brief Baud rate options (unified from all motor types)
   */
  enum class BaudRate : int
  {
    BAUD_2400 = 2400,
    BAUD_4800 = 4800,
    BAUD_9600 = 9600,
    BAUD_19200 = 19200,
    BAUD_38400 = 38400,
    BAUD_57600 = 57600,
    BAUD_115200 = 115200,
    BAUD_256000 = 256000
  };

  /**
   * @brief Parity options
   */
  enum class Parity
  {
    NONE,   // No parity (8N1) - used by ZD Motor, LC Stepper
    EVEN,   // Even parity (EVEN-1) - used by LC Servo
    ODD     // Odd parity
  };

  /**
   * @brief Constructor
   * @param device_path Path to serial device (e.g., /dev/ttyUSB0)
   * @param baud_rate Baud rate for communication (default: 19200 for servo/ZD, 115200 for stepper)
   * @param parity Parity setting (default: NONE)
   * @param timeout_ms Read timeout in milliseconds
   */
  RS485Client(
    const std::string & device_path,
    BaudRate baud_rate = BaudRate::BAUD_19200,
    Parity parity = Parity::NONE,
    int timeout_ms = 1000
  );

  /**
   * @brief Destructor - closes serial port if open
   */
  ~RS485Client();

  // Non-copyable
  RS485Client(const RS485Client &) = delete;
  RS485Client & operator=(const RS485Client &) = delete;

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
   * @brief Get device path
   * @return Device path string
   */
  std::string getDevicePath() const;

  // ==================== Modbus Function Codes ====================

  /**
   * @brief Read holding registers (Function Code 0x03)
   * @param slave_address Slave device address
   * @param start_address Starting register address
   * @param num_registers Number of registers to read
   * @param result Vector to store read values (16-bit each)
   * @return true if successful, false otherwise
   */
  bool readHoldingRegisters(
    uint8_t slave_address,
    uint16_t start_address,
    uint16_t num_registers,
    std::vector<uint16_t> & result
  );

  /**
   * @brief Read input registers (Function Code 0x04)
   * @param slave_address Slave device address
   * @param start_address Starting register address
   * @param num_registers Number of registers to read
   * @param result Vector to store read values (16-bit each)
   * @return true if successful, false otherwise
   */
  bool readInputRegisters(
    uint8_t slave_address,
    uint16_t start_address,
    uint16_t num_registers,
    std::vector<uint16_t> & result
  );

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

  // ==================== Low-level Interface ====================

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

  // ==================== Configuration ====================

  /**
   * @brief Set timeout for read operations
   * @param timeout_ms Timeout in milliseconds
   */
  void setTimeout(int timeout_ms);

  /**
   * @brief Get timeout for read operations
   * @return Timeout in milliseconds
   */
  int getTimeout() const;

  /**
   * @brief Get last error message
   * @return Error message string
   */
  std::string getLastError() const;

private:
  /**
   * @brief Calculate CRC16 for Modbus RTU communication
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
   * @brief Configure serial port with appropriate parity settings
   * @return true if successful
   */
  bool configureSerialPort();

  std::string device_path_;
  BaudRate baud_rate_;
  Parity parity_;
  int timeout_ms_;
  int serial_fd_;
  bool is_open_;
  mutable std::mutex mutex_;
  std::string last_error_;
};

// Type alias for backward compatibility with old servo motor code
using RS485ClientServo = RS485Client;

}  // namespace rs485_interface

#endif  // RS485_COMMON_CLIENT_HPP

