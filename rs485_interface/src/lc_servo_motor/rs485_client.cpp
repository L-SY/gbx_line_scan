#include "rs485_interface/lc_servo_motor/rs485_client.hpp"

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
  size_t expected_length = 8;  // 1+1+2+2+2(CRC)
  std::vector<uint8_t> response;
  if (!receiveFrame(expected_length, response)) {
    return false;
  }

  // Verify response matches request (echo check)
  if (response.size() < 6) {
    last_error_ = "Invalid response length";
    return false;
  }

  if (response[0] != slave_address || response[1] != 0x06) {
    last_error_ = "Response does not match request";
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
  auto inter_char_timeout = std::chrono::milliseconds(50);  // Max time between characters

  while (bytes_read < total_expected) {
    // Check overall timeout
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed > timeout_duration) {
      last_error_ = "Read timeout";
      return false;
    }

    // Use select with a short timeout
    fd_set read_fds;
    struct timeval select_timeout;
    
    FD_ZERO(&read_fds);
    FD_SET(serial_fd_, &read_fds);
    
    // Use inter-character timeout for remaining bytes, longer for first byte
    long timeout_ms = (bytes_read == 0) ? timeout_ms_ : 50;
    select_timeout.tv_sec = timeout_ms / 1000;
    select_timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int select_result = select(serial_fd_ + 1, &read_fds, nullptr, nullptr, &select_timeout);
    if (select_result < 0) {
      last_error_ = "Select error: " + std::string(strerror(errno));
      return false;
    }
    if (select_result == 0) {
      // Timeout - if we have some data, check if frame is complete
      if (bytes_read > 0) {
        // Check if we have enough data (at least address + function code + CRC)
        if (bytes_read >= 4) {
          break;  // Try to verify what we have
        }
      }
      last_error_ = "Read timeout";
      return false;
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

  // Check if we have minimum required bytes
  if (response.size() < 4) {
    last_error_ = "Incomplete frame received";
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

