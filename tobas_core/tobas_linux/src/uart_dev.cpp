#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "../include/tobas_linux/uart_dev.hpp"

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

  // Set raw mode
  options_.c_iflag = 0;
  options_.c_oflag = 0;
  options_.c_lflag = 0;

  // Set control mode flags
  options_.c_cflag &= ~CSIZE;   // Clear data bit size
  options_.c_cflag |= CS8;      // 8 bit size
  options_.c_cflag &= ~CSTOPB;  // 1 stop bit
  options_.c_cflag |= CREAD;    // Enable receiver
  options_.c_cflag &= ~PARENB;  // No parity
  options_.c_cflag &= ~HUPCL;   // No hung-up
  options_.c_cflag |= CLOCAL;   // Set local mode

  // Set minimum characters
  options_.c_cc[VMIN] = 0;   // Wait for 0 characters
  options_.c_cc[VTIME] = 0;  // Infinite timeout

  // Set baud rate
  options_.c_cflag &= ~CBAUD;            // Remove current baud rate
  options_.c_cflag |= BOTHER;            // Allow custom baud rate using int input
  options_.c_ispeed = kDefaultBaudRate;  // Set the input baud rate
  options_.c_ospeed = kDefaultBaudRate;  // Set the output baud rate

  // Set the new configuration of the serial interface
  if (!setConfig())
    return false;

  return true;
}

bool UARTdev::setBaudRate(uint32_t baud_rate)
{
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
  if (write(uart_fd_, data, length) != static_cast<ssize_t>(length))
  {
    cerr << "UART TX error." << endl;
    return false;
  }
  return true;
}

bool UARTdev::receive(uint8_t* data, size_t length)
{
  if (read(uart_fd_, data, length) != static_cast<ssize_t>(length))
  {
    cerr << "UART RX error." << endl;
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
