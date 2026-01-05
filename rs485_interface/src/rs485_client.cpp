#include "rs485_interface/rs485_client.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <algorithm>

namespace rs485_interface
{

RS485Client::RS485Client(
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

RS485Client::~RS485Client()
{
  close();
}

bool RS485Client::open()
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

void RS485Client::close()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  is_open_ = false;
}

bool RS485Client::isOpen() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_open_;
}

bool RS485Client::writeSingleRegister(
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

bool RS485Client::writeSingleRegisterNoResponse(
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

  // Don't wait for response - just send and return
  last_error_.clear();
  return true;
}

bool RS485Client::writeMultipleRegisters(
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

bool RS485Client::writeMultipleRegistersNoResponse(
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

  // Don't wait for response - just send and return
  last_error_.clear();
  return true;
}

bool RS485Client::sendFrame(const std::vector<uint8_t> & frame)
{
  if (frame.empty()) {
    last_error_ = "Empty frame";
    return false;
  }

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

  return true;
}

bool RS485Client::receiveFrame(size_t expected_length, std::vector<uint8_t> & response)
{
  response.clear();
  response.reserve(expected_length + 2);  // +2 for CRC

  // Use select for timeout
  fd_set read_fds;
  struct timeval timeout;

  FD_ZERO(&read_fds);
  FD_SET(serial_fd_, &read_fds);

  timeout.tv_sec = timeout_ms_ / 1000;
  timeout.tv_usec = (timeout_ms_ % 1000) * 1000;

  size_t total_expected = expected_length + 2;  // +2 for CRC
  size_t bytes_read = 0;

  while (bytes_read < total_expected) {
    int select_result = select(serial_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (select_result < 0) {
      last_error_ = "Select error: " + std::string(strerror(errno));
      return false;
    }
    if (select_result == 0) {
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
      }
    }

    // Reset timeout for next iteration
    timeout.tv_sec = timeout_ms_ / 1000;
    timeout.tv_usec = (timeout_ms_ % 1000) * 1000;
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

void RS485Client::setTimeout(int timeout_ms)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timeout_ms_ = timeout_ms;
}

std::string RS485Client::getLastError() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

uint16_t RS485Client::calculateCRC16(const uint8_t * data, size_t length) const
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

bool RS485Client::verifyCRC(const std::vector<uint8_t> & frame) const
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

bool RS485Client::configureSerialPort()
{
  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    last_error_ = "Failed to get serial attributes: " + std::string(strerror(errno));
    return false;
  }

  // Set baud rate
  speed_t speed;
  switch (baud_rate_) {
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
    case BaudRate::BAUD_115200:
      speed = B115200;
      break;
    case BaudRate::BAUD_256000:
      speed = B115200;  // Will attempt custom baud rate below
      break;
    default:
      speed = B115200;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);
  
  // Set custom baud rate for 256000 if needed
  if (baud_rate_ == BaudRate::BAUD_256000) {
    struct serial_struct ss;
    if (ioctl(serial_fd_, TIOCGSERIAL, &ss) == 0) {
      ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
      ss.custom_divisor = (ss.baud_base + (256000 / 2)) / 256000;
      if (ss.custom_divisor < 1) {
        ss.custom_divisor = 1;
      }
      ioctl(serial_fd_, TIOCSSERIAL, &ss);
    }
  }

  // 8 data bits, 1 stop bit, no parity (8N1)
  tty.c_cflag &= ~PARENB;   // No parity
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag &= ~CSIZE;    // Clear size bits
  tty.c_cflag |= CS8;       // 8 data bits

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

