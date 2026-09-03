// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/uart_dev.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <expected>
#include <iostream>
#include <thread>

#include "tobas_linux/error.hpp"
#include "tobas_linux/termios2.hpp"

using namespace std;

namespace tobas
{
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

bool UARTdev::initialize(const char* uart_dev, bool block_mode) noexcept
{
  block_mode_ = block_mode;

  // Close UART device if already opened.
  if (uart_fd_ >= 0) {
    close(uart_fd_);
  }

  // Open UART device.
  int oflag = O_RDWR | O_NOCTTY;
  if (!block_mode) {
    oflag |= O_NONBLOCK;
  }
  uart_fd_ = open(uart_dev, oflag);
  if (uart_fd_ < 0) {
    cerr << "Failed to open UART device \"" << uart_dev << "\": " << strError() << endl;
    return false;
  }

  // Get the current configuration of the serial interface.
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

  // Ignore control characters.
  for (auto& c_cc : options_.c_cc) {
    c_cc = 0;
  }

  // Set minimum characters.
  options_.c_cc[VMIN] = block_mode ? 1 : 0;

  // Set the new configuration of the serial interface.
  if (!setConfig()) {
    return false;
  }

  // Reset input buffer.
  if (tcflush(uart_fd_, TCIFLUSH) != 0) {
    cerr << "Failed to reset input buffer: " << strError() << endl;
    return false;
  }

  return true;
}

bool UARTdev::setBaudRate(uint32_t baud_rate) noexcept
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

bool UARTdev::enableParity(ParityMode mode)
{
  switch (mode) {
    case kOdd:
      options_.c_cflag |= PARODD;
      break;
    case kEven:
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
  if (!block_mode_ && msec_100 > 0) {
    cerr << "The timeout configuration is disabled in non-blocking mode." << endl;
    return false;
  }
  options_.c_cc[VTIME] = msec_100;  // FIXME: Not reflected
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

UARTdev::ReceiveResult UARTdev::tryReceiveByte(bool _nonblock) noexcept
{
  if (uart_fd_ < 0) {
    return unexpected(ReceiveError::kDeviceError);
  }

  struct pollfd poll_fd = { uart_fd_, POLLIN, 0 };

  while (true) {
    poll_fd.revents = 0;
    const auto poll_result = poll(&poll_fd, 1, _nonblock ? 0 : -1);
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }

      cerr << "UART RX poll failed: " << strError() << endl;
      return unexpected(ReceiveError::kDeviceError);
    }
    if (poll_result == 0) {
      return unexpected(ReceiveError::kNoData);
    }

    if ((poll_fd.revents & POLLNVAL) != 0) {
      cerr << "UART RX poll reported a device error." << endl;
      return unexpected(ReceiveError::kDeviceError);
    }

    if ((poll_fd.revents & POLLIN) != 0) {
      uint8_t data;
      const auto read_result = ::read(uart_fd_, &data, 1);
      if (read_result == 1) {
        return data;
      }
      if (read_result == 0) {
        cerr << "UART RX reached the end of the device stream." << endl;
        return unexpected(ReceiveError::kDeviceError);
      }
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        if (_nonblock) {
          return unexpected(ReceiveError::kNoData);
        }
        continue;
      }

      cerr << "UART RX failed: " << strError() << endl;
      return unexpected(ReceiveError::kDeviceError);
    }

    if ((poll_fd.revents & POLLERR) != 0) {
      cerr << "UART RX poll reported a device error." << endl;
      return unexpected(ReceiveError::kDeviceError);
    }

    if ((poll_fd.revents & POLLHUP) != 0) {
      cerr << "UART RX device hung up." << endl;
      return unexpected(ReceiveError::kDeviceError);
    }

    if (_nonblock) {
      return unexpected(ReceiveError::kNoData);
    }
  }
}

bool UARTdev::sendAll(const uint8_t* _data, size_t _length) noexcept
{
  size_t sent_length = 0;
  while (sent_length < _length) {
    const auto write_result = ::write(uart_fd_, _data + sent_length, _length - sent_length);
    if (write_result > 0) {
      sent_length += static_cast<size_t>(write_result);
      continue;
    }
    if (write_result == 0) {
      cerr << "UART TX transmitted zero bytes." << endl;
      return false;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      cerr << "UART TX failed: " << strError() << endl;
      return false;
    }

    struct pollfd poll_fd = { uart_fd_, POLLOUT, 0 };
    while (true) {
      const auto poll_result = poll(&poll_fd, 1, -1);
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }

        cerr << "UART TX poll failed: " << strError() << endl;
        return false;
      }
      if ((poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        cerr << "UART TX poll reported a device error." << endl;
        return false;
      }
      if ((poll_fd.revents & POLLOUT) != 0) {
        break;
      }
    }
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

bool UARTdev::getConfig() noexcept
{
  if (tcgetattr(uart_fd_, &options_) != 0) {
    cerr << "Failed to get serial port settings." << endl;
    return false;
  }
  return true;
}

bool UARTdev::setConfig() noexcept
{
  if (tcsetattr(uart_fd_, TCSANOW, &options_) != 0) {
    cerr << "Failed to set serial port settings." << endl;
    return false;
  }

  this_thread::sleep_for(1ms);

  return true;
}

bool UARTdev::isStandardBaudRate(uint32_t baud_rate) noexcept
{
  return baudrate_constants_.contains(baud_rate);
}

bool UARTdev::setStandardBaudRate(uint32_t baud_rate) noexcept
{
  const auto& flag = baudrate_constants_.at(baud_rate);

  options_.c_cflag &= ~CBAUD;
  options_.c_cflag |= flag;
  options_.c_ispeed = flag;
  options_.c_ospeed = flag;

  return setConfig();
}

bool UARTdev::setNonStandardBaudRate(uint32_t baud_rate) noexcept
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
}  // namespace tobas
