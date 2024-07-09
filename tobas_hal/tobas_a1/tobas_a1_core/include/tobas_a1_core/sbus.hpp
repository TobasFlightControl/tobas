#pragma once

#include <tobas_linux/uart_dev.hpp>

namespace a1
{
class SBUS
{
  static constexpr char kUartDev[] = "/dev/serial0";
  static constexpr uint32_t kBaudRate = 100000;  // [bps]
  static constexpr uint8_t kDataBits = 8;
  static constexpr uint8_t kDataSize = 22;
  static constexpr uint8_t kChannelBits = 11;
  static constexpr uint8_t kChannelSize = 16;

public:
  explicit SBUS();

  bool initialize();

  bool update();

private:
  linux::UARTdev uart_dev_;
  uint8_t data_[kDataSize];
  uint16_t periods_[kChannelSize];

  bool read();
  void decode();
};
}  // namespace a1
