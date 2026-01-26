#include "microswitch/serial_reader.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>

namespace microswitch
{

SerialReader::SerialReader(const std::string & device_path, int baud_rate)
: device_path_(device_path),
  baud_rate_(baud_rate),
  serial_fd_(-1),
  is_open_(false)
{
  buffer_.reserve(4096);
}

SerialReader::~SerialReader()
{
  close();
}

bool SerialReader::open()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (is_open_) {
    last_error_ = "Port already open";
    return false;
  }

  serial_fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    last_error_ = "Failed to open " + device_path_ + ": " + strerror(errno);
    return false;
  }

  if (!configureSerialPort()) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Keep non-blocking mode for readAvailable
  // int flags = fcntl(serial_fd_, F_GETFL, 0);
  // if (flags >= 0) {
  //   fcntl(serial_fd_, F_SETFL, flags & ~O_NONBLOCK);
  // }

  is_open_ = true;
  last_error_.clear();
  return true;
}

void SerialReader::close()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  is_open_ = false;
}

bool SerialReader::isOpen() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return is_open_;
}

bool SerialReader::readLine(std::string & line, int timeout_ms)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_open_ || serial_fd_ < 0) {
    last_error_ = "Serial port not open";
    return false;
  }

  line.clear();
  char buffer[256];
  fd_set read_fds;
  struct timeval timeout;

  while (true) {
    FD_ZERO(&read_fds);
    FD_SET(serial_fd_, &read_fds);

    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int select_result = select(serial_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (select_result < 0) {
      last_error_ = "select failed: " + std::string(strerror(errno));
      return false;
    }
    if (select_result == 0) {
      last_error_ = "Read timeout";
      return false;
    }

    if (FD_ISSET(serial_fd_, &read_fds)) {
      ssize_t bytes_read = ::read(serial_fd_, buffer, sizeof(buffer) - 1);
      if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        last_error_ = "read failed: " + std::string(strerror(errno));
        return false;
      }
      if (bytes_read == 0) {
        continue;
      }

      buffer[bytes_read] = '\0';
      line += buffer;

      size_t newline_pos = line.find('\n');
      if (newline_pos != std::string::npos) {
        line = line.substr(0, newline_pos);
        size_t cr_pos = line.find('\r');
        if (cr_pos != std::string::npos) {
          line = line.substr(0, cr_pos);
        }
        last_error_.clear();
        return true;
      }
    }
  }
}

bool SerialReader::readAvailable(std::vector<std::string> & lines)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_open_ || serial_fd_ < 0) {
    last_error_ = "Serial port not open";
    return false;
  }

  lines.clear();
  char read_buffer[1024];
  
  // Read all available data (non-blocking)
  while (true) {
    ssize_t bytes_read = ::read(serial_fd_, read_buffer, sizeof(read_buffer) - 1);
    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      last_error_ = "read failed: " + std::string(strerror(errno));
      return false;
    }
    if (bytes_read == 0) {
      break;
    }

    read_buffer[bytes_read] = '\0';
    buffer_ += read_buffer;
  }

  // Parse complete lines from buffer
  size_t pos = 0;
  while (true) {
    size_t newline_pos = buffer_.find('\n', pos);
    if (newline_pos == std::string::npos) {
      break;
    }

    std::string line = buffer_.substr(pos, newline_pos - pos);
    // Remove carriage return if present
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (!line.empty()) {
      lines.push_back(line);
    }
    pos = newline_pos + 1;
  }

  // Keep remaining data in buffer
  if (pos > 0) {
    buffer_ = buffer_.substr(pos);
  }

  last_error_.clear();
  return true;
}

std::string SerialReader::getLastError() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

bool SerialReader::configureSerialPort()
{
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  if (tcgetattr(serial_fd_, &tty) != 0) {
    last_error_ = "tcgetattr failed: " + std::string(strerror(errno));
    return false;
  }

  speed_t speed = baudRateToSpeed(baud_rate_);
  if (cfsetospeed(&tty, speed) != 0 || cfsetispeed(&tty, speed) != 0) {
    last_error_ = "Failed to set baud rate";
    return false;
  }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    last_error_ = "tcsetattr failed: " + std::string(strerror(errno));
    return false;
  }

  return true;
}

speed_t SerialReader::baudRateToSpeed(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      return B115200;
  }
}

}  // namespace microswitch
