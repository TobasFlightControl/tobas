#include <iostream>
#include <thread>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "../include/tobas_linux/uart_dev.hpp"
#include "../include/tobas_linux/errer.hpp"
#include "../include/tobas_linux/termios2.hpp"

using namespace std;

namespace linux
{
UARTdev::UARTdev()
  : baudrate_constants_{
      { 0, B0 },
      { 50, B50 },
      { 75, B75 },
      { 110, B110 },
      { 134, B134 },
      { 150, B150 },
      { 200, B200 },
      { 300, B300 },
      { 600, B600 },
      { 1200, B1200 },
      { 1800, B1800 },
      { 2400, B2400 },
      { 4800, B4800 },
      { 9600, B9600 },
      { 19200, B19200 },
      { 38400, B38400 },
      { 57600, B57600 },
      { 115200, B115200 },
      { 230400, B230400 },
      { 460800, B460800 },
      { 500000, B500000 },
      { 576000, B576000 },
      { 921600, B921600 },
      { 1000000, B1000000 },
      { 1152000, B1152000 },
      { 1500000, B1500000 },
      { 2000000, B2000000 },
      { 2500000, B2500000 },
      { 3000000, B3000000 },
      { 3500000, B3500000 },
      { 4000000, B4000000 },
    }
{
}

UARTdev::~UARTdev()
{
  if (uart_fd_ >= 0) {
    close(uart_fd_);
  }
}

bool UARTdev::initialize(const char* uart_dev, bool block_mode)
{
  block_mode_ = block_mode;

  // Close UART device if already opened
  if (uart_fd_ >= 0) {
    close(uart_fd_);
  }

  // Open UART device
  int oflag = O_RDWR | O_NOCTTY;
  if (!block_mode) {
    oflag |= O_NONBLOCK;
  }
  uart_fd_ = open(uart_dev, oflag);
  if (uart_fd_ < 0) {
    cerr << "Failed to open UART device \"" << uart_dev << "\": " << strError() << endl;
    return false;
  }

  // Get the current configuration of the serial interface
  if (!getConfig()) {
    return false;
  }

  // c_cflag bits
  options_.c_cflag &= ~CSIZE;    // Clear data bit size
  options_.c_cflag |= CS8;       // 8 bit size
  options_.c_cflag &= ~CSTOPB;   // 1 stop bit
  options_.c_cflag |= CREAD;     // Enable receiver
  options_.c_cflag &= ~PARENB;   // No parity
  options_.c_cflag &= ~HUPCL;    // No hung-up
  options_.c_cflag |= CLOCAL;    // Set local mode
  options_.c_cflag &= ~CMSPAR;   // Disable mark or space parity
  options_.c_cflag &= ~CRTSCTS;  // Disable RTS/CTS flow control

  // c_iflag bits
  options_.c_iflag = 0;

  // c_oflag bits
  options_.c_oflag = 0;

  // c_lflag bits
  options_.c_lflag = 0;

  // Ignore control characters
  for (auto& c_cc : options_.c_cc) {
    c_cc = 0;
  }

  // Set minimum characters
  options_.c_cc[VMIN] = block_mode ? 1 : 0;

  // Set the new configuration of the serial interface
  if (!setConfig()) {
    return false;
  }

  // Reset input buffer
  if (tcflush(uart_fd_, TCIFLUSH) != 0) {
    cerr << "Failed to reset input buffer: " << strError() << endl;
    return false;
  }

  return true;
}

bool UARTdev::setBaudRate(uint32_t baud_rate)
{
  if (isStandardBaudRate(baud_rate)) {
    return setStandardBaudRate(baud_rate);
  }
  else {
    return setNonStandardBaudRate(baud_rate);
  }
}

bool UARTdev::setDataBits(uint8_t data_bits)
{
  tcflag_t flag;
  switch (data_bits) {
    case 5:
      flag = CS5;
      break;
    case 6:
      flag = CS6;
      break;
    case 7:
      flag = CS7;
      break;
    case 8:
      flag = CS8;
      break;
    default:
      cerr << "Invalid data bit size." << endl;
      return false;
  }

  options_.c_cflag &= ~CSIZE;
  options_.c_cflag |= flag;
  return setConfig();
}

bool UARTdev::setSingleStopBit()
{
  options_.c_cflag &= ~CSTOPB;
  return setConfig();
}

bool UARTdev::setDoubleStopBit()
{
  options_.c_cflag |= CSTOPB;
  return setConfig();
}

bool UARTdev::enableParity(parity_mode_t mode)
{
  switch (mode) {
    case PARITY_ODD:
      options_.c_cflag |= PARODD;
      break;
    case PARITY_EVEN:
      options_.c_cflag &= ~PARODD;
      break;
    default:
      cerr << "Invalid parity mode." << endl;
      return false;
  }

  options_.c_cflag |= PARENB;
  options_.c_iflag |= IGNPAR;
  options_.c_iflag |= INPCK;

  return setConfig();
}

bool UARTdev::disableParity()
{
  options_.c_cflag &= ~PARENB;
  options_.c_iflag &= ~IGNPAR;
  options_.c_iflag &= ~INPCK;

  return setConfig();
}

bool UARTdev::enableHungupClose()
{
  options_.c_cflag |= HUPCL;
  return setConfig();
}

bool UARTdev::disableHungupClose()
{
  options_.c_cflag &= ~HUPCL;
  return setConfig();
}

bool UARTdev::setTimeout(cc_t msec_100)
{
  options_.c_cc[VTIME] = msec_100;
  return setConfig();
}

bool UARTdev::setMinimumChars(uint8_t num)
{
  if (!block_mode_ && num > 0) {
    cerr << "The minimum number of characters configuration is disabled in non-blocking mode." << endl;
    return false;
  }
  options_.c_cc[VMIN] = num;
  return setConfig();
}

bool UARTdev::send(const uint8_t* data, size_t length)
{
  const auto res = ::write(uart_fd_, data, length);
  if (res < 0) {
    cerr << "UART TX failed: " << strError() << endl;
    return false;
  }
  if (res != static_cast<ssize_t>(length)) {
    cerr << "Tried to transmit " << length << " bytes, but " << res << " bytes were transmitted." << endl;
    return false;
  }

  return true;
}

bool UARTdev::receive(uint8_t* data, size_t length)
{
  const auto res = ::read(uart_fd_, data, length);
  if (res < 0) {
    cerr << "UART RX failed: " << strError() << endl;
    return false;
  }
  if (res != static_cast<ssize_t>(length)) {
    cerr << "Tried to receive " << length << " bytes, but " << res << " bytes were received." << endl;
    return false;
  }

  return true;
}

uint8_t UARTdev::receiveByte()
{
  if (!block_mode_) {
    throw runtime_error("This method cannot be called in non-blocking mode.");
  }

  uint8_t byte;
  if (!receive(&byte, 1)) {
    throw runtime_error("Failed to receive 1 byte.");
  }

  return byte;
}

bool UARTdev::getConfig()
{
  if (tcgetattr(uart_fd_, &options_) != 0) {
    cerr << "Failed to get serial port settings." << endl;
    return false;
  }
  return true;
}

bool UARTdev::setConfig()
{
  if (tcsetattr(uart_fd_, TCSANOW, &options_) != 0) {
    cerr << "Failed to set serial port settings." << endl;
    return false;
  }

  this_thread::sleep_for(1ms);

  return true;
}

bool UARTdev::isStandardBaudRate(uint32_t baud_rate)
{
  return baudrate_constants_.contains(baud_rate);
}

bool UARTdev::setStandardBaudRate(uint32_t baud_rate)
{
  const auto& flag = baudrate_constants_.at(baud_rate);

  options_.c_cflag &= ~CBAUD;
  options_.c_cflag |= flag;
  options_.c_ispeed = flag;
  options_.c_ospeed = flag;

  return setConfig();
}

bool UARTdev::setNonStandardBaudRate(uint32_t baud_rate)
{
  options_.c_cflag &= ~CBAUD;
  options_.c_cflag |= CBAUDEX;
  options_.c_ispeed = CBAUDEX;
  options_.c_ospeed = CBAUDEX;

  if (!setConfig()) {
    return false;
  }

  if (!linux::setNonStandardBaudRate(uart_fd_, baud_rate)) {
    return false;
  }

  return true;
}
}  // namespace linux
