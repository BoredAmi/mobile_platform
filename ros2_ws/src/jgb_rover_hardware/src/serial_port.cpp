#include "jgb_rover_hardware/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace jgb_rover_hardware
{

SerialPort::~SerialPort() {close();}

bool SerialPort::open(const std::string & device)
{
  close();
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    error_ = device + ": " + std::strerror(errno);
    return false;
  }
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    error_ = device + ": not a serial port (" + std::strerror(errno) + ")";
    close();
    return false;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  // USB CDC ignores the rate, but the RP2040 SDK reboots into its bootloader at 1200 baud: never
  // use that one.
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    error_ = device + ": tcsetattr failed (" + std::strerror(errno) + ")";
    close();
    return false;
  }
  flush_input();
  error_.clear();
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

long SerialPort::read(uint8_t * buf, std::size_t len)
{
  if (fd_ < 0) {
    return -1;
  }
  ssize_t n = ::read(fd_, buf, len);
  if (n > 0) {
    return n;
  }
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
    return 0;
  }
  // n == 0 on a tty in non-blocking raw mode means nothing to read, unless the device vanished;
  // an unplugged CDC device returns -1 / EIO.
  if (n == 0) {
    return 0;
  }
  error_ = std::string("read: ") + std::strerror(errno);
  close();
  return -1;
}

bool SerialPort::write(const uint8_t * buf, std::size_t len)
{
  if (fd_ < 0) {
    return false;
  }
  ssize_t n = ::write(fd_, buf, len);
  if (n == static_cast<ssize_t>(len)) {
    return true;
  }
  if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
    error_ = std::string("write: ") + std::strerror(errno);
    close();
  }
  return false;
}

void SerialPort::flush_input()
{
  if (fd_ >= 0) {
    tcflush(fd_, TCIFLUSH);
  }
}

}  // namespace jgb_rover_hardware
