#ifndef MICROSWITCH_SERIAL_READER_HPP_
#define MICROSWITCH_SERIAL_READER_HPP_

#include <string>
#include <vector>
#include <mutex>
#include <termios.h>

namespace microswitch
{

class SerialReader
{
public:
  SerialReader(const std::string & device_path, int baud_rate);
  ~SerialReader();

  bool open();
  void close();
  bool isOpen() const;
  
  bool readLine(std::string & line, int timeout_ms = 1000);
  bool readAvailable(std::vector<std::string> & lines);
  std::string getLastError() const;

private:
  bool configureSerialPort();
  speed_t baudRateToSpeed(int baud_rate);

  std::string device_path_;
  int baud_rate_;
  int serial_fd_;
  bool is_open_;
  mutable std::mutex mutex_;
  std::string last_error_;
  std::string buffer_;
};

}  // namespace microswitch

#endif  // MICROSWITCH_SERIAL_READER_HPP_
