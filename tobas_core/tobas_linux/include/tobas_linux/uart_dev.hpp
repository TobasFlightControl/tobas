#pragma once

#include <cinttypes>
#include <cstddef>

namespace linux
{
class UARTdev
{
public:
  explicit UARTdev();
  ~UARTdev();

  bool initialize(const char* uart_dev, uint32_t baudrate);

  bool send(const uint8_t* data, size_t length);
  bool receive(uint8_t* data, size_t length);

private:
  int uart_fd_;
};
}  // namespace linux
