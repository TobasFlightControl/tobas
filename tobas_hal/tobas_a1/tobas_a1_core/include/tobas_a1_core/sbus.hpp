#pragma once

#include <tobas_linux/uart_dev.hpp>

namespace a1
{
/**
 * @brief インバータで信号を反転したS.BUSをUARTで読む．
 * cf. [Raspberry Pi PicoのUARTでラジコン受信機の信号を読む](https://rikei-tawamure.com/entry/2021/02/12/130248)
 */
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

  inline const uint16_t& getPeriod(uint8_t channel) const;

private:
  linux::UARTdev uart_dev_;
  uint8_t data_[kDataSize];
  uint16_t periods_[kChannelSize];

  bool read();
  void decode();
};

inline const uint16_t& SBUS::getPeriod(uint8_t channel) const
{
  assert(channel < kChannelSize);
  return periods_[channel];
}
}  // namespace a1
