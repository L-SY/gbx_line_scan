#include "xyt300_motor/modbus_client.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>
#include <chrono>
#include <thread>

namespace xyt300_motor
{

ModbusClient::ModbusClient(
  const std::string & device_path,
  BaudRate baud_rate,
  int timeout_ms)
: device_path_(device_path),
  baud_rate_(baud_rate),
  timeout_ms_(timeout_ms),
  serial_fd_(-1),
  is_open_(false)
{
}

ModbusClient::~ModbusClient()
{
  close();
}

bool ModbusClient::open()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (is_open_) {
    last_error_ = "Port already open";
    return false;
  }

  // Open serial port (non-blocking first)
  serial_fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    last_error_ = "Failed to open " + device_path_ + ": " + strerror(errno) + 
                  " (errno=" + std::to_string(errno) + ")";
    return false;
  }

  // Configure serial port
  if (!configureSerialPort()) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }
  
  // Set to blocking mode after configuration
  int flags = fcntl(serial_fd_, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(serial_fd_, F_SETFL, flags & ~O_NONBLOCK);
  }

  is_open_ = true;
  last_error_.clear();
  return true;
}

void ModbusClient::close()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  is_open_ = false;
}

bool ModbusClient::isOpen() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_open_;
}

std::string ModbusClient::getDevicePath() const
{
  return device_path_;
}

bool ModbusClient::configureSerialPort()
{
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  if (tcgetattr(serial_fd_, &tty) != 0) {
    last_error_ = "tcgetattr failed: " + std::string(strerror(errno));
    return false;
  }

  // Set baud rate
  speed_t speed;
  switch (baud_rate_) {
    case BaudRate::BAUD_2400: speed = B2400; break;
    case BaudRate::BAUD_4800: speed = B4800; break;
    case BaudRate::BAUD_9600: speed = B9600; break;
    case BaudRate::BAUD_19200: speed = B19200; break;
    case BaudRate::BAUD_38400: speed = B38400; break;
    case BaudRate::BAUD_57600: speed = B57600; break;
    case BaudRate::BAUD_115200: speed = B115200; break;
    case BaudRate::BAUD_256000: speed = B230400; break;  // Closest available
    default: speed = B115200; break;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8N1 configuration
  tty.c_cflag &= ~PARENB;  // No parity
  tty.c_cflag &= ~CSTOPB;  // 1 stop bit
  tty.c_cflag &= ~CSIZE;   // Clear size bits
  tty.c_cflag |= CS8;      // 8 data bits
  tty.c_cflag &= ~CRTSCTS; // No hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

  // Input flags
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  // Output flags
  tty.c_oflag &= ~OPOST;

  // Local flags
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  // Set timeouts
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    last_error_ = "tcsetattr failed: " + std::string(strerror(errno));
    return false;
  }

  // Flush buffers
  tcflush(serial_fd_, TCIOFLUSH);

  return true;
}

uint16_t ModbusClient::calculateCRC16(const uint8_t * data, size_t length) const
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool ModbusClient::verifyCRC(const std::vector<uint8_t> & frame) const
{
  if (frame.size() < 4) {
    return false;
  }

  size_t data_length = frame.size() - 2;
  uint16_t calculated_crc = calculateCRC16(frame.data(), data_length);
  uint16_t received_crc = static_cast<uint16_t>(frame[data_length]) |
                          (static_cast<uint16_t>(frame[data_length + 1]) << 8);

  return calculated_crc == received_crc;
}

bool ModbusClient::sendData(const std::vector<uint8_t> & data)
{
  // Clear any pending input data before sending
  tcflush(serial_fd_, TCIFLUSH);
  
  ssize_t written = ::write(serial_fd_, data.data(), data.size());
  if (written < 0) {
    last_error_ = "Write failed: " + std::string(strerror(errno));
    return false;
  }
  if (static_cast<size_t>(written) != data.size()) {
    last_error_ = "Incomplete write";
    return false;
  }
  tcdrain(serial_fd_);  // Wait for transmission to complete
  
  // Small delay to ensure data is sent
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  
  return true;
}

bool ModbusClient::receiveData(size_t expected_length, std::vector<uint8_t> & data)
{
  data.clear();
  data.reserve(expected_length + 2);  // +2 for CRC

  auto start_time = std::chrono::steady_clock::now();
  size_t target_length = expected_length + 2;
  size_t no_data_count = 0;
  const size_t MAX_NO_DATA_COUNT = 20;  // Allow more iterations with no data
  bool first_byte_received = false;

  while (data.size() < target_length) {
    // Check timeout
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    
    // If we haven't received first byte, use shorter timeout
    int effective_timeout = first_byte_received ? timeout_ms_ : timeout_ms_ / 2;
    
    if (elapsed_ms > effective_timeout) {
      last_error_ = "Receive timeout (expected " + std::to_string(target_length) + 
                    " bytes, received " + std::to_string(data.size()) + " bytes)";
      return false;
    }

    // Use select to check if data is available
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(serial_fd_, &read_fds);

    struct timeval tv;
    int remaining_ms = effective_timeout - static_cast<int>(elapsed_ms);
    if (remaining_ms <= 0) {
      remaining_ms = 50;  // At least 50ms
    }
    tv.tv_sec = remaining_ms / 1000;
    tv.tv_usec = (remaining_ms % 1000) * 1000;

    int select_result = select(serial_fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (select_result < 0) {
      last_error_ = "Select failed: " + std::string(strerror(errno));
      return false;
    }
    if (select_result == 0) {
      // No data available
      if (data.size() >= target_length) {
        break;
      }
      no_data_count++;
      if (no_data_count > MAX_NO_DATA_COUNT) {
        // If we have some data but not enough, wait a bit more
        if (data.size() > 0) {
          std::this_thread::sleep_for(std::chrono::milliseconds(30));
          no_data_count = 0;
          continue;
        }
        last_error_ = "Receive timeout (no data received after " + 
                      std::to_string(elapsed_ms) + "ms)";
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // Read available data
    uint8_t buffer[256];
    ssize_t n = ::read(serial_fd_, buffer, sizeof(buffer));
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      last_error_ = "Read failed: " + std::string(strerror(errno));
      return false;
    }

    if (n > 0) {
      data.insert(data.end(), buffer, buffer + n);
      if (!first_byte_received) {
        first_byte_received = true;
        // Reset timeout counter since we got first byte
        start_time = std::chrono::steady_clock::now();
      }
      no_data_count = 0;  // Reset counter when we receive data
    } else {
      no_data_count++;
      if (no_data_count > MAX_NO_DATA_COUNT && data.size() == 0) {
        last_error_ = "No data received";
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // Check if we received enough data
  if (data.size() < target_length) {
    last_error_ = "Incomplete response (expected " + std::to_string(target_length) + 
                  " bytes, received " + std::to_string(data.size()) + " bytes)";
    return false;
  }

  return true;
}

bool ModbusClient::readHoldingRegisters(
  uint8_t slave_address,
  uint16_t start_address,
  uint16_t num_registers,
  std::vector<uint16_t> & result)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_open_) {
    last_error_ = "Port not open";
    return false;
  }

  if (num_registers == 0 || num_registers > 125) {
    last_error_ = "Invalid number of registers (1-125)";
    return false;
  }

  // Build request frame: [Slave Address] [Function Code 0x03] [Start Address High] [Start Address Low] [Quantity High] [Quantity Low] [CRC Low] [CRC High]
  std::vector<uint8_t> request;
  request.push_back(slave_address);
  request.push_back(0x03);  // Function code: Read Holding Registers
  request.push_back(static_cast<uint8_t>((start_address >> 8) & 0xFF));
  request.push_back(static_cast<uint8_t>(start_address & 0xFF));
  request.push_back(static_cast<uint8_t>((num_registers >> 8) & 0xFF));
  request.push_back(static_cast<uint8_t>(num_registers & 0xFF));

  // Calculate and append CRC
  uint16_t crc = calculateCRC16(request.data(), request.size());
  request.push_back(static_cast<uint8_t>(crc & 0xFF));
  request.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

  // Clear input buffer before sending
  tcflush(serial_fd_, TCIFLUSH);
  
  // Send request
  if (!sendData(request)) {
    return false;
  }

  // Wait for response (MODBUS RTU requires 3.5 character times)
  // At 115200 baud: 3.5 * 11 bits / 115200 = ~0.33ms, but we wait longer for safety
  // For slower devices, we need more time
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // Receive response: [Slave Address] [Function Code] [Byte Count] [Data...] [CRC Low] [CRC High]
  size_t expected_length = 3 + num_registers * 2;  // 3 bytes header + 2 bytes per register
  std::vector<uint8_t> response;
  if (!receiveData(expected_length, response)) {
    return false;
  }

  // Verify CRC
  if (!verifyCRC(response)) {
    last_error_ = "CRC error";
    return false;
  }

  // Verify response
  if (response.size() < 5) {
    last_error_ = "Response too short";
    return false;
  }

  if (response[0] != slave_address) {
    last_error_ = "Slave address mismatch";
    return false;
  }

  if (response[1] != 0x03) {
    if (response[1] & 0x80) {
      // Error response
      last_error_ = "MODBUS error code: " + std::to_string(response[2]);
      return false;
    }
    last_error_ = "Unexpected function code";
    return false;
  }

  uint8_t byte_count = response[2];
  if (byte_count != num_registers * 2) {
    last_error_ = "Byte count mismatch";
    return false;
  }

  // Extract register values
  result.clear();
  result.reserve(num_registers);
  for (size_t i = 0; i < num_registers; ++i) {
    size_t offset = 3 + i * 2;
    uint16_t value = static_cast<uint16_t>(response[offset]) << 8 |
                     static_cast<uint16_t>(response[offset + 1]);
    result.push_back(value);
  }

  last_error_.clear();
  return true;
}

bool ModbusClient::readInputRegisters(
  uint8_t slave_address,
  uint16_t start_address,
  uint16_t num_registers,
  std::vector<uint16_t> & result)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_open_) {
    last_error_ = "Port not open";
    return false;
  }

  if (num_registers == 0 || num_registers > 125) {
    last_error_ = "Invalid number of registers (1-125)";
    return false;
  }

  // Build request frame: [Slave Address] [Function Code 0x04] [Start Address High] [Start Address Low] [Quantity High] [Quantity Low] [CRC Low] [CRC High]
  std::vector<uint8_t> request;
  request.push_back(slave_address);
  request.push_back(0x04);  // Function code: Read Input Registers
  request.push_back(static_cast<uint8_t>((start_address >> 8) & 0xFF));
  request.push_back(static_cast<uint8_t>(start_address & 0xFF));
  request.push_back(static_cast<uint8_t>((num_registers >> 8) & 0xFF));
  request.push_back(static_cast<uint8_t>(num_registers & 0xFF));

  // Calculate and append CRC
  uint16_t crc = calculateCRC16(request.data(), request.size());
  request.push_back(static_cast<uint8_t>(crc & 0xFF));
  request.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

  // Clear input buffer before sending
  tcflush(serial_fd_, TCIFLUSH);
  
  // Send request
  if (!sendData(request)) {
    return false;
  }

  // Wait for response
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  // Receive response: [Slave Address] [Function Code] [Byte Count] [Data...] [CRC Low] [CRC High]
  size_t expected_length = 3 + num_registers * 2;  // 3 bytes header + 2 bytes per register
  std::vector<uint8_t> response;
  if (!receiveData(expected_length, response)) {
    return false;
  }

  // Verify CRC
  if (!verifyCRC(response)) {
    last_error_ = "CRC error";
    return false;
  }

  // Verify response
  if (response.size() < 5) {
    last_error_ = "Response too short";
    return false;
  }

  if (response[0] != slave_address) {
    last_error_ = "Slave address mismatch";
    return false;
  }

  if (response[1] != 0x04) {
    if (response[1] & 0x80) {
      // Error response
      last_error_ = "MODBUS error code: " + std::to_string(response[2]);
      return false;
    }
    last_error_ = "Unexpected function code";
    return false;
  }

  uint8_t byte_count = response[2];
  if (byte_count != num_registers * 2) {
    last_error_ = "Byte count mismatch";
    return false;
  }

  // Extract register values
  result.clear();
  result.reserve(num_registers);
  for (size_t i = 0; i < num_registers; ++i) {
    size_t offset = 3 + i * 2;
    uint16_t value = (static_cast<uint16_t>(response[offset]) << 8) |
                     static_cast<uint16_t>(response[offset + 1]);
    result.push_back(value);
  }

  last_error_.clear();
  return true;
}

bool ModbusClient::writeSingleRegister(
  uint8_t slave_address,
  uint16_t register_address,
  uint16_t value)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_open_) {
    last_error_ = "Port not open";
    return false;
  }

  // Build request frame: [Slave Address] [Function Code 0x06] [Register Address High] [Register Address Low] [Value High] [Value Low] [CRC Low] [CRC High]
  // According to protocol: register address and data use big-endian (high byte first, low byte last)
  std::vector<uint8_t> request;
  request.push_back(slave_address);
  request.push_back(0x06);  // Function code: Write Single Register
  request.push_back(static_cast<uint8_t>((register_address >> 8) & 0xFF));  // Register address high byte
  request.push_back(static_cast<uint8_t>(register_address & 0xFF));          // Register address low byte
  request.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));             // Value high byte first
  request.push_back(static_cast<uint8_t>(value & 0xFF));                    // Value low byte last

  // Calculate and append CRC
  uint16_t crc = calculateCRC16(request.data(), request.size());
  request.push_back(static_cast<uint8_t>(crc & 0xFF));        // CRC low byte first
  request.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF)); // CRC high byte last

  // Clear input buffer before sending
  tcflush(serial_fd_, TCIFLUSH);
  
  // Send request
  if (!sendData(request)) {
    return false;
  }

  // Wait for response
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // Receive response (echo of request)
  // Response format: [Slave Address] [Function Code] [Reg Addr High] [Reg Addr Low] [Value High] [Value Low] [CRC Low] [CRC High]
  // Total: 8 bytes (6 data bytes + 2 CRC bytes)
  // receiveData expects data length without CRC, so we pass 6
  std::vector<uint8_t> response;
  if (!receiveData(6, response)) {
    return false;
  }

  // Verify CRC
  if (!verifyCRC(response)) {
    last_error_ = "CRC error";
    return false;
  }

  // Verify response matches request
  if (response.size() < 8) {
    last_error_ = "Response too short";
    return false;
  }

  if (response[0] != slave_address || response[1] != 0x06) {
    if (response[1] & 0x80) {
      last_error_ = "MODBUS error code: " + std::to_string(response[2]);
      return false;
    }
    last_error_ = "Response mismatch";
    return false;
  }

  last_error_.clear();
  return true;
}

bool ModbusClient::writeSingleRegisterNoResponse(
  uint8_t slave_address,
  uint16_t register_address,
  uint16_t value)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_open_) {
    last_error_ = "Port not open";
    return false;
  }

  // Build request frame: [Slave Address] [Function Code 0x06] [Register Address High] [Register Address Low] [Value High] [Value Low] [CRC Low] [CRC High]
  // According to protocol: register address and data use big-endian (high byte first, low byte last)
  std::vector<uint8_t> request;
  request.push_back(slave_address);
  request.push_back(0x06);  // Function code: Write Single Register
  request.push_back(static_cast<uint8_t>((register_address >> 8) & 0xFF));  // Register address high byte
  request.push_back(static_cast<uint8_t>(register_address & 0xFF));          // Register address low byte
  request.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));             // Value high byte first
  request.push_back(static_cast<uint8_t>(value & 0xFF));                    // Value low byte last

  // Calculate and append CRC
  uint16_t crc = calculateCRC16(request.data(), request.size());
  request.push_back(static_cast<uint8_t>(crc & 0xFF));        // CRC low byte first
  request.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF)); // CRC high byte last

  // Clear input buffer before sending
  tcflush(serial_fd_, TCIFLUSH);
  
  // Send request (no response expected)
  if (!sendData(request)) {
    return false;
  }

  last_error_.clear();
  return true;
}

bool ModbusClient::writeMultipleRegisters(
  uint8_t slave_address,
  uint16_t start_address,
  const std::vector<uint16_t> & values)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_open_) {
    last_error_ = "Port not open";
    return false;
  }

  if (values.empty() || values.size() > 123) {
    last_error_ = "Invalid number of registers (1-123)";
    return false;
  }

  uint16_t num_registers = static_cast<uint16_t>(values.size());
  uint8_t byte_count = static_cast<uint8_t>(num_registers * 2);

  // Build request frame: [Slave Address] [Function Code 0x10] [Start Address High] [Start Address Low] 
  // [Quantity High] [Quantity Low] [Byte Count] [Data...] [CRC Low] [CRC High]
  std::vector<uint8_t> request;
  request.push_back(slave_address);
  request.push_back(0x10);  // Function code: Write Multiple Registers
  request.push_back(static_cast<uint8_t>((start_address >> 8) & 0xFF));
  request.push_back(static_cast<uint8_t>(start_address & 0xFF));
  request.push_back(static_cast<uint8_t>((num_registers >> 8) & 0xFF));
  request.push_back(static_cast<uint8_t>(num_registers & 0xFF));
  request.push_back(byte_count);

  // Add register values (high byte first, low byte last for each register)
  for (const auto & value : values) {
    request.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    request.push_back(static_cast<uint8_t>(value & 0xFF));
  }

  // Calculate and append CRC
  uint16_t crc = calculateCRC16(request.data(), request.size());
  request.push_back(static_cast<uint8_t>(crc & 0xFF));
  request.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

  // Clear input buffer before sending
  tcflush(serial_fd_, TCIFLUSH);
  
  // Send request
  if (!sendData(request)) {
    return false;
  }

  // Wait for response
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // Receive response: [Slave Address] [Function Code] [Start Address High] [Start Address Low] 
  // [Quantity High] [Quantity Low] [CRC Low] [CRC High]
  // Total: 8 bytes (6 data bytes + 2 CRC bytes)
  std::vector<uint8_t> response;
  if (!receiveData(6, response)) {
    return false;
  }

  // Verify CRC
  if (!verifyCRC(response)) {
    last_error_ = "CRC error";
    return false;
  }

  // Verify response
  if (response.size() < 8) {
    last_error_ = "Response too short";
    return false;
  }

  if (response[0] != slave_address || response[1] != 0x10) {
    if (response[1] & 0x80) {
      last_error_ = "MODBUS error code: " + std::to_string(response[2]);
      return false;
    }
    last_error_ = "Response mismatch";
    return false;
  }

  // Verify start address and quantity match
  uint16_t resp_start = (static_cast<uint16_t>(response[2]) << 8) | response[3];
  uint16_t resp_quantity = (static_cast<uint16_t>(response[4]) << 8) | response[5];
  if (resp_start != start_address || resp_quantity != num_registers) {
    last_error_ = "Response address/quantity mismatch";
    return false;
  }

  last_error_.clear();
  return true;
}

void ModbusClient::setTimeout(int timeout_ms)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timeout_ms_ = timeout_ms;
}

int ModbusClient::getTimeout() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return timeout_ms_;
}

std::string ModbusClient::getLastError() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

}  // namespace xyt300_motor
