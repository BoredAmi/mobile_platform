// Minimal non-blocking POSIX serial port for the RP2040's USB CDC device.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace jgb_rover_hardware
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Opens in raw mode, non-blocking. Returns false and sets error() on failure.
  bool open(const std::string & device);
  void close();
  bool is_open() const {return fd_ >= 0;}

  // Reads what is available (never blocks). Returns bytes read, 0 if none, -1 if the device is
  // gone (unplugged); the port is then closed.
  long read(uint8_t * buf, std::size_t len);
  // Writes the whole buffer or nothing useful: returns false if the device is gone or the
  // kernel buffer is full (the frame is dropped, the next cycle sends a fresh one).
  bool write(const uint8_t * buf, std::size_t len);
  void flush_input();

  const std::string & error() const {return error_;}

private:
  int fd_{-1};
  std::string error_;
};

}  // namespace jgb_rover_hardware
