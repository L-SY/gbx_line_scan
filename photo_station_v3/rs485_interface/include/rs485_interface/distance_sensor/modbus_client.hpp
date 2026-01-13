#ifndef MODBUS_CLIENT_HPP
#define MODBUS_CLIENT_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <mutex>

namespace rs485_interface
{

/**
 * @brief MODBUS RTU client for RS485 communication
 * 
 * This class provides a base implementation for MODBUS RTU protocol
 * communication over RS485 serial interface. It handles serial port
 * configuration, MODBUS frame construction, CRC calculation, and
 * basic read/write operations.
 */
class ModbusClient
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
   * @brief MODBUS function codes
   */
  enum class FunctionCode : uint8_t
  {
    READ_HOLDING_REGISTERS = 0x04,  // Read Input Registers
    WRITE_SINGLE_REGISTER = 0x06,   // Write Single Register
    WRITE_MULTIPLE_REGISTERS = 0x10 // Write Multiple Registers
  };

  /**
   * @brief Constructor
   * @param device_path Path to serial device (e.g., /dev/ttyACM0)
   * @param baud_rate Baud rate for communication
   * @param timeout_ms Read timeout in milliseconds
   */
  ModbusClient(
    const std::string & device_path,
    BaudRate baud_rate = BaudRate::BAUD_115200,
    int timeout_ms = 1000
  );

  /**
   * @brief Destructor - closes serial port if open
   */
  ~ModbusClient();

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
   * @brief Read holding registers (Function Code 0x04)
   * @param slave_address MODBUS slave address
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
   * @brief Write single register (Function Code 0x06)
   * @param slave_address MODBUS slave address
   * @param register_address Register address to write
   * @param value Value to write
   * @return true if successful, false otherwise
   */
  bool writeSingleRegister(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t value
  );

  /**
   * @brief Write multiple registers (Function Code 0x10)
   * @param slave_address MODBUS slave address
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
   * @brief Calculate CRC16 for MODBUS RTU
   * @param data Data buffer
   * @param length Data length
   * @return CRC16 value
   */
  uint16_t calculateCRC16(const uint8_t * data, size_t length) const;

  /**
   * @brief Send MODBUS request frame
   * @param frame Frame data (without CRC, will be added)
   * @return true if successful
   */
  bool sendFrame(const std::vector<uint8_t> & frame);

  /**
   * @brief Receive MODBUS response frame
   * @param expected_length Expected frame length
   * @param response Received frame data
   * @return true if successful
   */
  bool receiveFrame(size_t expected_length, std::vector<uint8_t> & response);

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

#endif  // MODBUS_CLIENT_HPP

