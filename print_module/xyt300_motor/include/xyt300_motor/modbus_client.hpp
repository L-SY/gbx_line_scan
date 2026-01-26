#ifndef MODBUS_CLIENT_HPP
#define MODBUS_CLIENT_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include <mutex>

namespace xyt300_motor
{

/**
 * @brief Simple MODBUS RTU client for RS485 communication
 */
class ModbusClient
{
public:
  /**
   * @brief Baud rate options
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
   * @brief Constructor
   * @param device_path Path to serial device (e.g., /dev/ttyACM0)
   * @param baud_rate Baud rate for communication (default: 115200)
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

  // Non-copyable
  ModbusClient(const ModbusClient &) = delete;
  ModbusClient & operator=(const ModbusClient &) = delete;

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
   * @brief Write single register without waiting for response (for initialization)
   * @param slave_address Slave device address
   * @param register_address Register address to write
   * @param value Value to write (16-bit)
   * @return true if send successful, false otherwise
   * 
   * This method is used for initialization commands that don't require a response.
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
   * @param values Vector of values to write (16-bit each)
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
   * @brief Configure serial port
   * @return true if successful
   */
  bool configureSerialPort();

  /**
   * @brief Send data
   * @param data Data to send
   * @return true if successful
   */
  bool sendData(const std::vector<uint8_t> & data);

  /**
   * @brief Receive data
   * @param expected_length Expected frame length
   * @param data Received data
   * @return true if successful
   */
  bool receiveData(size_t expected_length, std::vector<uint8_t> & data);

  std::string device_path_;
  BaudRate baud_rate_;
  int timeout_ms_;
  int serial_fd_;
  bool is_open_;
  mutable std::mutex mutex_;
  std::string last_error_;
};

}  // namespace xyt300_motor

#endif  // MODBUS_CLIENT_HPP
