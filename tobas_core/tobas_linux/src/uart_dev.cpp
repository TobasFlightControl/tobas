#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "../include/tobas_linux/uart_dev.hpp"

using namespace std;

namespace linux
{
UARTdev::UARTdev()
{
}

UARTdev::~UARTdev()
{
  close(uart_fd_);
}

bool UARTdev::initialize(const char* uart_dev, uint32_t baudrate)
{
  uart_fd_ = open(uart_dev, O_RDWR | O_NOCTTY | O_NDELAY);
  if (uart_fd_ < 0)
  {
    cerr << "Failed to open UART device: " << uart_dev << endl;
    return false;
  }

  termios options;
  tcgetattr(uart_fd_, &options);
  options.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;

  tcflush(uart_fd_, TCIFLUSH);
  tcsetattr(uart_fd_, TCSANOW, &options);

  return true;
}

bool UARTdev::send(const uint8_t* data, size_t length)
{
  if (write(uart_fd_, data, length) != length)
  {
    cerr << "UART TX error." << endl;
    return false;
  }
  return true;
}

bool UARTdev::receive(uint8_t* data, size_t length)
{
  if (read(uart_fd_, data, length) != length)
  {
    cerr << "UART RX error." << endl;
    return false;
  }
  return true;
}
}  // namespace linux
