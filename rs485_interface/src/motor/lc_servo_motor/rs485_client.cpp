#include "rs485_interface/motor/lc_servo_motor/rs485_client.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>

namespace rs485_interface
{

RS485ClientServo::RS485ClientServo(
  const std::string & device_path,
  BaudRate baud_rate,
  Parity parity,
  int timeout_ms)
: device_path_(device_path),
  baud_rate_(baud_rate),
  parity_(parity),
  timeout_ms_(timeout_ms),
  serial_fd_(-1),
  is_open_(false)
{
}

RS485ClientServo::~RS485ClientServo()
{
  close();
}

bool RS485ClientServo::open()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (is_open_) {
    last_error_ = "Port already open";
    return false;
  }

  // Open serial port in non-blocking mode
  serial_fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    last_error_ = "Failed to open " + device_path_ + ": " + strerror(errno);
    return false;
  }

  // Configure serial port
  if (!configureSerialPort()) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  is_open_ = true;
  last_error_.clear();
  return true;
}

void RS485ClientServo::close()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  is_open_ = false;
}

bool RS485ClientServo::isOpen() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_open_;
}

bool RS485ClientServo::readHoldingRegisters(
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

  // Build request frame: [SlaveAddr][0x03][StartAddrHi][StartAddrLo][NumRegHi][NumRegLo]
  std::vector<uint8_t> frame;
  frame.push_back(slave_address);
  frame.push_back(0x03);  // Function Code: Read Holding Registers
  frame.push_back((start_address >> 8) & 0xFF);
  frame.push_back(start_address & 0xFF);
  frame.push_back((num_registers >> 8) & 0xFF);
  frame.push_back(num_registers & 0xFF);

  if (!sendFrame(frame)) {
    return false;
  }

  // Expected response: [SlaveAddr][0x03][ByteCount][Data...][CRCHi][CRCLo]
  // ByteCount = num_registers * 2
  size_t expected_length = 3 + num_registers * 2;  // 1+1+1+data+2(CRC)
  std::vector<uint8_t> response;
  if (!receiveFrame(expected_length, response)) {
    return false;
  }

  // Verify response
  if (response.size() < 3) {
    last_error_ = "Invalid response length";
    return false;
  }

  if (response[0] != slave_address || response[1] != 0x03) {
    last_error_ = "Response does not match request";
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
    uint16_t value = (response[3 + i * 2] << 8) | response[3 + i * 2 + 1];
    result.push_back(value);
  }

  last_error_.clear();
  return true;
}

bool RS485ClientServo::writeSingleRegister(
  uint8_t slave_address,
  uint16_t register_address,
  uint16_t value)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_open_) {
    last_error_ = "Port not open";
    return false;
  }

  // Build request frame: [SlaveAddr][0x06][RegAddrHi][RegAddrLo][ValueHi][ValueLo]
  std::vector<uint8_t> frame;
  frame.push_back(slave_address);
  frame.push_back(0x06);  // Function Code: Write Single Register
  frame.push_back((register_address >> 8) & 0xFF);
  frame.push_back(register_address & 0xFF);
  frame.push_back((value >> 8) & 0xFF);
  frame.push_back(value & 0xFF);

  if (!sendFrame(frame)) {
    return false;
  }

  // Expected echo response: [SlaveAddr][0x06][RegAddrHi][RegAddrLo][ValueHi][ValueLo][CRCHi][CRCLo]
  // Total: 6 data bytes + 2 CRC bytes = 8 bytes
  // Exception response: [SlaveAddr][0x86][ErrorCode][CRCHi][CRCLo] = 5 bytes
  size_t expected_data_length = 6;  // Data bytes only (without CRC)
  std::vector<uint8_t> response;
  if (!receiveFrame(expected_data_length, response)) {
    return false;
  }

  // Check for exception response (5 bytes total: 3 data + 2 CRC)
  if (response.size() == 3) {
    if (response[0] == slave_address && response[1] == 0x86) {
      // Exception response: function code 0x86 = 0x06 + 0x80
      uint8_t error_code = response[2];
      std::string error_msg = "Modbus exception response: error code 0x";
      char hex[3];
      snprintf(hex, sizeof(hex), "%02X", error_code);
      error_msg += hex;
      error_msg += " (";
      switch (error_code) {
        case 0x01: error_msg += "Illegal Function"; break;
        case 0x02: error_msg += "Illegal Data Address"; break;
        case 0x03: error_msg += "Illegal Data Value"; break;
        case 0x04: error_msg += "Slave Device Failure"; break;
        case 0x06: error_msg += "Slave Device Busy"; break;
        default: error_msg += "Unknown error"; break;
      }
      error_msg += ")";
      last_error_ = error_msg;
      return false;
    }
  }

  // Verify response matches request (echo check)
  // Response should have 6 bytes: [SlaveAddr][0x06][RegAddrHi][RegAddrLo][ValueHi][ValueLo]
  if (response.size() < 6) {
    last_error_ = "Invalid response length: got " + std::to_string(response.size()) + 
                  " bytes, expected 6 (normal) or 3 (exception)";
    return false;
  }

  if (response[0] != slave_address) {
    last_error_ = "Response slave address mismatch: got " + std::to_string(response[0]) + 
                  ", expected " + std::to_string(slave_address);
    return false;
  }

  if (response[1] != 0x06) {
    last_error_ = "Response function code mismatch: got 0x" + 
                  std::to_string(response[1]) + ", expected 0x06";
    return false;
  }

  // Verify register address matches
  uint16_t resp_reg_addr = (response[2] << 8) | response[3];
  if (resp_reg_addr != register_address) {
    last_error_ = "Response register address mismatch";
    return false;
  }

  // Verify value matches
  uint16_t resp_value = (response[4] << 8) | response[5];
  if (resp_value != value) {
    last_error_ = "Response value mismatch";
    return false;
  }

  last_error_.clear();
  return true;
}

bool RS485ClientServo::writeMultipleRegisters(
  uint8_t slave_address,
  uint16_t start_address,
  const std::vector<uint16_t> & values)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_open_) {
    last_error_ = "Port not open";
    return false;
  }

  if (values.empty()) {
    last_error_ = "Empty values vector";
    return false;
  }

  // Build request frame: [SlaveAddr][0x10][StartAddrHi][StartAddrLo][NumRegHi][NumRegLo][ByteCount][Data...]
  std::vector<uint8_t> frame;
  frame.push_back(slave_address);
  frame.push_back(0x10);  // Function Code: Write Multiple Registers
  frame.push_back((start_address >> 8) & 0xFF);
  frame.push_back(start_address & 0xFF);
  frame.push_back((values.size() >> 8) & 0xFF);
  frame.push_back(values.size() & 0xFF);
  frame.push_back(values.size() * 2);  // Byte count

  for (uint16_t value : values) {
    frame.push_back((value >> 8) & 0xFF);
    frame.push_back(value & 0xFF);
  }

  if (!sendFrame(frame)) {
    return false;
  }

  // Expected response: [SlaveAddr][0x10][StartAddrHi][StartAddrLo][NumRegHi][NumRegLo][CRCHi][CRCLo]
  size_t expected_length = 8;  // 1+1+2+2+2(CRC)
  std::vector<uint8_t> response;
  if (!receiveFrame(expected_length, response)) {
    return false;
  }

  // Verify response
  if (response.size() < 6) {
    last_error_ = "Invalid response length";
    return false;
  }

  if (response[0] != slave_address || response[1] != 0x10) {
    last_error_ = "Response does not match request";
    return false;
  }

  last_error_.clear();
  return true;
}

void RS485ClientServo::setTimeout(int timeout_ms)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timeout_ms_ = timeout_ms;
}

std::string RS485ClientServo::getLastError() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

uint16_t RS485ClientServo::calculateCRC16(const uint8_t * data, size_t length) const
{
  uint16_t crc = 0xFFFF;
  const uint16_t polynomial = 0xA001;

  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ polynomial;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

bool RS485ClientServo::verifyCRC(const std::vector<uint8_t> & frame) const
{
  if (frame.size() < 2) {
    return false;
  }

  // Calculate CRC for all bytes except last 2 (CRC bytes)
  uint16_t calculated_crc = calculateCRC16(frame.data(), frame.size() - 2);

  // Extract received CRC (low byte first, high byte second)
  uint16_t received_crc = frame[frame.size() - 2] | (frame[frame.size() - 1] << 8);

  return calculated_crc == received_crc;
}

bool RS485ClientServo::sendFrame(const std::vector<uint8_t> & frame)
{
  if (frame.empty()) {
    last_error_ = "Empty frame";
    return false;
  }

  // Clear receive buffer before sending
  tcflush(serial_fd_, TCIFLUSH);

  // Calculate CRC
  uint16_t crc = calculateCRC16(frame.data(), frame.size());

  // Append CRC (low byte first, high byte second)
  std::vector<uint8_t> full_frame = frame;
  full_frame.push_back(crc & 0xFF);        // CRC Low
  full_frame.push_back((crc >> 8) & 0xFF); // CRC High

  // Write to serial port
  ssize_t written = ::write(serial_fd_, full_frame.data(), full_frame.size());
  if (written != static_cast<ssize_t>(full_frame.size())) {
    last_error_ = "Failed to write frame: " + std::string(strerror(errno));
    return false;
  }

  // Flush to ensure data is sent
  tcdrain(serial_fd_);

  // Modbus RTU requires a frame delay (3.5 character times)
  // For 19200 baud: 3.5 * (11 bits / 19200) ≈ 2ms
  // Add a small safety margin
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  return true;
}

bool RS485ClientServo::receiveFrame(size_t expected_length, std::vector<uint8_t> & response)
{
  response.clear();
  response.reserve(expected_length + 2);  // +2 for CRC

  size_t total_expected = expected_length + 2;  // +2 for CRC
  size_t bytes_read = 0;
  
  // Use a longer timeout for Modbus RTU
  // Wait for first byte with longer timeout, then shorter timeout for remaining bytes
  auto start_time = std::chrono::steady_clock::now();
  auto timeout_duration = std::chrono::milliseconds(timeout_ms_);
  
  // Wait a bit longer after sending to allow device to process
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  while (bytes_read < total_expected) {
    // Check overall timeout
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed > timeout_duration) {
      if (bytes_read > 0) {
        // We got some data but not enough, return what we have
        break;
      }
      last_error_ = "Read timeout: no data received";
      return false;
    }

    // Use select with a timeout
    fd_set read_fds;
    struct timeval select_timeout;
    
    FD_ZERO(&read_fds);
    FD_SET(serial_fd_, &read_fds);
    
    // Use longer timeout for first byte, shorter for subsequent bytes
    long timeout_ms = (bytes_read == 0) ? timeout_ms_ : 100;
    select_timeout.tv_sec = timeout_ms / 1000;
    select_timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int select_result = select(serial_fd_ + 1, &read_fds, nullptr, nullptr, &select_timeout);
    if (select_result < 0) {
      last_error_ = "Select error: " + std::string(strerror(errno));
      return false;
    }
    if (select_result == 0) {
      // Timeout - if we have some data, check if frame might be complete
      if (bytes_read >= 4) {
        // We have at least address + function code + some data, might be complete
        break;
      }
      // No data yet, continue waiting if we haven't exceeded overall timeout
      continue;
    }

    if (FD_ISSET(serial_fd_, &read_fds)) {
      uint8_t buffer[256];
      ssize_t n = ::read(serial_fd_, buffer, sizeof(buffer));
      if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          last_error_ = "Read error: " + std::string(strerror(errno));
          return false;
        }
      } else if (n > 0) {
        response.insert(response.end(), buffer, buffer + n);
        bytes_read += n;
        // Reset start time when we receive data
        start_time = std::chrono::steady_clock::now();
      }
    }
  }

  // Check if we have minimum required bytes (address + function code + CRC)
  if (response.size() < 4) {
    last_error_ = "Incomplete frame received: got " + std::to_string(response.size()) + 
                  " bytes, minimum 4 required";
    return false;
  }

  // Check if this might be an exception response (3 data bytes + 2 CRC = 5 bytes total)
  // Exception response format: [SlaveAddr][FunctionCode+0x80][ErrorCode][CRCHi][CRCLo]
  if (response.size() == 5) {
    // Verify CRC first
    if (verifyCRC(response)) {
      // Valid exception response, remove CRC and return it for handling
      response.resize(response.size() - 2);
      return true;
    }
  }

  // Verify we have enough bytes for the expected data + CRC
  if (response.size() < expected_length + 2) {
    // Try to verify CRC with what we have
    if (response.size() >= 4) {
      // Check if we can verify CRC on partial frame
      std::vector<uint8_t> partial_frame(response.begin(), response.end());
      if (verifyCRC(partial_frame)) {
        // CRC is valid, but frame is shorter than expected
        // This might be an error response or shorter frame
        // Remove CRC and return it for handling
        response.resize(response.size() - 2);
        return true;
      }
    }
    last_error_ = "Incomplete frame: got " + std::to_string(response.size()) + 
                  " bytes, expected " + std::to_string(expected_length + 2);
    return false;
  }

  // Verify CRC
  if (!verifyCRC(response)) {
    last_error_ = "CRC verification failed";
    return false;
  }

  // Remove CRC bytes from response
  response.resize(response.size() - 2);

  return true;
}

bool RS485ClientServo::configureSerialPort()
{
  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    last_error_ = "Failed to get serial attributes: " + std::string(strerror(errno));
    return false;
  }

  // Set baud rate
  speed_t speed;
  switch (baud_rate_) {
    case BaudRate::BAUD_2400:
      speed = B2400;
      break;
    case BaudRate::BAUD_4800:
      speed = B4800;
      break;
    case BaudRate::BAUD_9600:
      speed = B9600;
      break;
    case BaudRate::BAUD_19200:
      speed = B19200;
      break;
    case BaudRate::BAUD_38400:
      speed = B38400;
      break;
    case BaudRate::BAUD_57600:
      speed = B57600;
      break;
    default:
      speed = B19200;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8 data bits, 1 stop bit
  tty.c_cflag &= ~CSIZE;    // Clear size bits
  tty.c_cflag |= CS8;       // 8 data bits
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit

  // Set parity based on configuration
  if (parity_ == Parity::EVEN) {
    tty.c_cflag |= PARENB;   // Enable parity
    tty.c_cflag &= ~PARODD;  // Even parity
  } else if (parity_ == Parity::ODD) {
    tty.c_cflag |= PARENB;   // Enable parity
    tty.c_cflag |= PARODD;   // Odd parity
  } else {
    tty.c_cflag &= ~PARENB;  // No parity
  }

  // Disable hardware flow control
  tty.c_cflag &= ~CRTSCTS;

  // Enable receiver
  tty.c_cflag |= CREAD | CLOCAL;

  // Disable canonical mode, echo, and signals
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  // Disable software flow control
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  // Raw output
  tty.c_oflag &= ~OPOST;

  // Set read timeout
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = timeout_ms_ / 100;  // Timeout in tenths of seconds

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    last_error_ = "Failed to set serial attributes: " + std::string(strerror(errno));
    return false;
  }

  // Set to blocking mode after configuration
  int flags = fcntl(serial_fd_, F_GETFL);
  flags &= ~O_NONBLOCK;
  fcntl(serial_fd_, F_SETFL, flags);

  return true;
}

}  // namespace rs485_interface

