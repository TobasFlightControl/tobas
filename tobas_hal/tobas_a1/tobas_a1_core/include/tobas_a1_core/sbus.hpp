#pragma once

#include <array>

#include <tobas_linux/uart_dev.hpp>

namespace a1
{
/**
 * @brief インバータで信号を反転したS.BUSをUARTで読む．
 * cf. [Raspberry Pi PicoのUARTでラジコン受信機の信号を読む](https://rikei-tawamure.com/entry/2021/02/12/130248)
 */
class SBUS
{
public:
  static constexpr uint8_t kChannelSize = 16;

private:
  static constexpr uint32_t kBaudRate = 100'000;  // [bps]
  static constexpr uint8_t kDataBits = 8;
  static constexpr uint8_t kDataSize = 22;
  static constexpr uint8_t kChannelBits = 11;

public:
  explicit SBUS();

  bool initialize();

  /* Read a S.BUS message from the receiver and update the periods of all the 16 channels. */
  bool update();

  inline const uint16_t& getPeriod(uint8_t channel) const;

private:
  linux::UARTdev uart_dev_;
  std::array<uint8_t, kDataSize> data_;
  std::array<uint16_t, kChannelSize> periods_;

  bool read();
  void decode();
};

inline const uint16_t& SBUS::getPeriod(uint8_t channel) const
{
  return periods_.at(channel);
}
}  // namespace a1
