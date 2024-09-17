#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "../include/tobas_linux/uart_dev.hpp"
#include "../include/tobas_linux/termios.hpp"
#include "../include/tobas_linux/errer.hpp"

using namespace std;

namespace linux
{
UARTdev::UARTdev()
{
}

UARTdev::~UARTdev()
{
  if (uart_fd_ >= 0)
    close(uart_fd_);
}

bool UARTdev::initialize(const char* uart_dev, bool block_mode)
{
  block_mode_ = block_mode;

  // Open UART device
  int oflag = O_RDWR | O_NOCTTY;
  if (!block_mode)
    oflag |= O_NONBLOCK;
  uart_fd_ = open(uart_dev, oflag);
  if (uart_fd_ < 0)
  {
    cerr << "Failed to open UART device: " << uart_dev << endl;
    return false;
  }

  // Get the current configuration of the serial interface
  if (!getConfig())
    return false;

  // Set flags. See termbits.h and termbits-common.h.

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
  options_.c_iflag |= IGNBRK;  // Ignore BREAK condition
  options_.c_iflag |= IGNPAR;  // Ignore characters with parity errors
  options_.c_iflag |= INPCK;   // Enable parity checking

  // c_oflag bits
  options_.c_oflag = 0;

  // c_lflag bits
  options_.c_lflag = 0;

  // Ignore control characters
  for (auto& c_cc : options_.c_cc)
    c_cc = 0;

  // Set minimum characters
  options_.c_cc[VMIN] = block_mode ? 1 : 0;

  // Set the new configuration of the serial interface
  if (!setConfig())
    return false;

  // Reset input buffer
  if (tcflush(uart_fd_, TCIFLUSH) != 0)
    return false;

  return true;
}

bool UARTdev::setStandardBaudRate(uint32_t baud_rate_flag)
{
  options_.c_cflag &= ~CBAUD;          // Remove current baud rate
  options_.c_cflag |= baud_rate_flag;  // Set baud rate flag

  // Clear non-standard baud rates
  options_.c_ispeed = 0;
  options_.c_ospeed = 0;

  return setConfig();
}

bool UARTdev::setNonStandardBaudRate(uint32_t baud_rate)
{
  options_.c_cflag &= ~CBAUD;   // Remove current baud rate
  options_.c_cflag |= CBAUDEX;  // Allow non-standard baud rate using int input

  // Set baud rate
  options_.c_ispeed = baud_rate;
  options_.c_ospeed = baud_rate;

  return setConfig();
}

bool UARTdev::setDataBits(uint8_t data_bits)
{
  tcflag_t flag;
  switch (data_bits)
  {
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
  switch (mode)
  {
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
  return setConfig();
}

bool UARTdev::disableParity()
{
  options_.c_cflag &= ~PARENB;
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

bool UARTdev::setMinimumChars(uint8_t num)
{
  if (!block_mode_ && num > 0)
  {
    cerr << "The minimum number of characters configuration is disabled in non-blocking mode." << endl;
    return false;
  }
  options_.c_cc[VMIN] = num;
  return setConfig();
}

bool UARTdev::send(const uint8_t* data, size_t length)
{
  const auto res = ::write(uart_fd_, data, length);
  if (res < 0)
  {
    cerr << "UART TX failed: " << strError() << endl;
    return false;
  }
  if (res != static_cast<ssize_t>(length))
  {
    cerr << "Tried to transmit " << length << " bytes, but " << res << " bytes were transmitted." << endl;
    return false;
  }

  return true;
}

bool UARTdev::receive(uint8_t* data, size_t length)
{
  const auto res = ::read(uart_fd_, data, length);
  if (res < 0)
  {
    cerr << "UART RX failed: " << strError() << endl;
    return false;
  }
  if (res != static_cast<ssize_t>(length))
  {
    cerr << "Tried to receive " << length << " bytes, but " << res << " bytes were received." << endl;
    return false;
  }

  return true;
}

bool UARTdev::getConfig()
{
  if (ioctl(uart_fd_, TCGETS2, &options_) < 0)
  {
    cerr << "Failed to get serial port settings." << endl;
    return false;
  }
  return true;
}

bool UARTdev::setConfig()
{
  if (ioctl(uart_fd_, TCSETS2, &options_) < 0)
  {
    cerr << "Failed to set serial port settings." << endl;
    return false;
  }
  return true;
}
}  // namespace linux
