#include "rs485_interface/modbus_client.hpp"

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

  // Build request frame: [SlaveAddr][FuncCode][StartAddrHi][StartAddrLo][NumRegHi][NumRegLo][CRCHi][CRCLo]
  std::vector<uint8_t> request;
  request.push_back(slave_address);
  request.push_back(static_cast<uint8_t>(FunctionCode::READ_HOLDING_REGISTERS));
  request.push_back((start_address >> 8) & 0xFF);
  request.push_back(start_address & 0xFF);
  request.push_back((num_registers >> 8) & 0xFF);
  request.push_back(num_registers & 0xFF);

  // Calculate and append CRC
  uint16_t crc = calculateCRC16(request.data(), request.size());
  request.push_back(crc & 0xFF);        // CRC Low
  request.push_back((crc >> 8) & 0xFF); // CRC High

  // Send request
  if (!sendFrame(request)) {
    return false;
  }

  // Expected response: [SlaveAddr][FuncCode][ByteCount][Data...][CRCHi][CRCLo]
  // ByteCount = num_registers * 2
  size_t expected_length = 5 + num_registers * 2;  // 1+1+1+data+2(CRC)
  std::vector<uint8_t> response;
  if (!receiveFrame(expected_length, response)) {
    return false;
  }

  // Verify response
  if (response.size() < 5) {
    last_error_ = "Response too short";
    return false;
  }

  if (response[0] != slave_address) {
    last_error_ = "Slave address mismatch in response";
    return false;
  }

  if (response[1] != static_cast<uint8_t>(FunctionCode::READ_HOLDING_REGISTERS)) {
    last_error_ = "Function code mismatch in response";
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

  // Build request frame: [SlaveAddr][FuncCode][RegAddrHi][RegAddrLo][ValueHi][ValueLo][CRCHi][CRCLo]
  std::vector<uint8_t> request;
  request.push_back(slave_address);
  request.push_back(static_cast<uint8_t>(FunctionCode::WRITE_SINGLE_REGISTER));
  request.push_back((register_address >> 8) & 0xFF);
  request.push_back(register_address & 0xFF);
  request.push_back((value >> 8) & 0xFF);
  request.push_back(value & 0xFF);

  // Calculate and append CRC
  uint16_t crc = calculateCRC16(request.data(), request.size());
  request.push_back(crc & 0xFF);        // CRC Low
  request.push_back((crc >> 8) & 0xFF); // CRC High

  // Send request
  if (!sendFrame(request)) {
    return false;
  }

  // Expected echo response: [SlaveAddr][FuncCode][RegAddrHi][RegAddrLo][ValueHi][ValueLo][CRCHi][CRCLo]
  std::vector<uint8_t> response;
  if (!receiveFrame(8, response)) {
    return false;
  }

  // Verify response (should echo the request)
  if (response.size() < 8) {
    last_error_ = "Response too short";
    return false;
  }

  if (response[0] != slave_address ||
      response[1] != static_cast<uint8_t>(FunctionCode::WRITE_SINGLE_REGISTER) ||
      (response[2] << 8 | response[3]) != register_address ||
      (response[4] << 8 | response[5]) != value)
  {
    last_error_ = "Response verification failed";
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

  if (values.empty() || values.size() > 123) {  // MODBUS limit
    last_error_ = "Invalid number of registers";
    return false;
  }

  uint16_t num_registers = static_cast<uint16_t>(values.size());

  // Build request frame
  std::vector<uint8_t> request;
  request.push_back(slave_address);
  request.push_back(static_cast<uint8_t>(FunctionCode::WRITE_MULTIPLE_REGISTERS));
  request.push_back((start_address >> 8) & 0xFF);
  request.push_back(start_address & 0xFF);
  request.push_back((num_registers >> 8) & 0xFF);
  request.push_back(num_registers & 0xFF);
  request.push_back(num_registers * 2);  // Byte count

  // Add values
  for (uint16_t value : values) {
    request.push_back((value >> 8) & 0xFF);
    request.push_back(value & 0xFF);
  }

  // Calculate and append CRC
  uint16_t crc = calculateCRC16(request.data(), request.size());
  request.push_back(crc & 0xFF);        // CRC Low
  request.push_back((crc >> 8) & 0xFF); // CRC High

  // Send request
  if (!sendFrame(request)) {
    return false;
  }

  // Expected response: [SlaveAddr][FuncCode][StartAddrHi][StartAddrLo][NumRegHi][NumRegLo][CRCHi][CRCLo]
  std::vector<uint8_t> response;
  if (!receiveFrame(8, response)) {
    return false;
  }

  // Verify response
  if (response.size() < 8) {
    last_error_ = "Response too short";
    return false;
  }

  if (response[0] != slave_address ||
      response[1] != static_cast<uint8_t>(FunctionCode::WRITE_MULTIPLE_REGISTERS) ||
      (response[2] << 8 | response[3]) != start_address ||
      (response[4] << 8 | response[5]) != num_registers)
  {
    last_error_ = "Response verification failed";
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

std::string ModbusClient::getLastError() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

uint16_t ModbusClient::calculateCRC16(const uint8_t * data, size_t length) const
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

bool ModbusClient::sendFrame(const std::vector<uint8_t> & frame)
{
  // Clear input buffer before sending
  tcflush(serial_fd_, TCIFLUSH);

  ssize_t written = ::write(serial_fd_, frame.data(), frame.size());
  if (written < 0) {
    last_error_ = "Write failed: " + std::string(strerror(errno));
    return false;
  }

  if (static_cast<size_t>(written) != frame.size()) {
    last_error_ = "Partial write";
    return false;
  }

  // Wait for transmission to complete (important for RS485)
  tcdrain(serial_fd_);

  // Small delay for frame transmission
  usleep(10000);  // 10ms

  return true;
}

bool ModbusClient::receiveFrame(size_t expected_length, std::vector<uint8_t> & response)
{
  response.clear();
  response.reserve(expected_length);

  struct timeval timeout;
  timeout.tv_sec = timeout_ms_ / 1000;
  timeout.tv_usec = (timeout_ms_ % 1000) * 1000;

  size_t bytes_received = 0;
  uint8_t buffer[256];

  while (bytes_received < expected_length) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(serial_fd_, &read_fds);

    int select_result = select(serial_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (select_result < 0) {
      last_error_ = "Select failed: " + std::string(strerror(errno));
      return false;
    }

    if (select_result == 0) {
      last_error_ = "Read timeout";
      return false;
    }

    if (FD_ISSET(serial_fd_, &read_fds)) {
      ssize_t n = ::read(serial_fd_, buffer, sizeof(buffer));
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        last_error_ = "Read failed: " + std::string(strerror(errno));
        return false;
      }

      if (n == 0) {
        continue;
      }

      response.insert(response.end(), buffer, buffer + n);
      bytes_received += n;

      // Reset timeout for inter-byte timeout (MODBUS RTU requirement)
      timeout.tv_sec = 0;
      timeout.tv_usec = 3500;  // 3.5 character times at worst case
    }
  }

  // Verify CRC
  if (!verifyCRC(response)) {
    last_error_ = "CRC verification failed";
    return false;
  }

  return true;
}

bool ModbusClient::verifyCRC(const std::vector<uint8_t> & frame) const
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

bool ModbusClient::configureSerialPort()
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
      // B256000 may not be available on all systems, use custom baud rate
      speed = B115200;  // Will be overridden with custom baud rate below
      break;
    default:
      speed = B115200;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);
  
  // Set custom baud rate for 256000 if needed
  // Note: Custom baud rates may not work on all systems
  if (baud_rate_ == BaudRate::BAUD_256000) {
    struct serial_struct ss;
    if (ioctl(serial_fd_, TIOCGSERIAL, &ss) == 0) {
      ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
      ss.custom_divisor = (ss.baud_base + (256000 / 2)) / 256000;
      if (ss.custom_divisor < 1) {
        ss.custom_divisor = 1;
      }
      ioctl(serial_fd_, TIOCSSERIAL, &ss);
      // Note: If custom baud rate fails, standard 115200 will be used
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

