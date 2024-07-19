#pragma once

#include <cstddef>

#include <tobas_linux/spi_dev.hpp>

namespace a1
{
class PWM
{
public:
  static constexpr size_t kChannelSize = 8;

private:
  static constexpr size_t kChannelBytes = 2;                           // 1チャネルあたりのバイト数
  static constexpr size_t kSpiBufSize = kChannelSize * kChannelBytes;  // SPIバッファのサイズ
  static constexpr uint32_t kSpiClockSpeed = 27'000'000;               // [Hz] F722のSPI2の最大値
  static constexpr uint16_t kThrottleMask = (1 << 11) - 1;

public:
  explicit PWM();

  bool initialize();

  inline void setPeriod(size_t ch, uint16_t period_us);
  inline bool transfer();

private:
  linux::SPIdev spi_;
  uint8_t tx_[kSpiBufSize] = { 0 };
  uint8_t rx_[kSpiBufSize] = { 0 };

  inline void setData(size_t ch, uint16_t data);
};

inline void PWM::setPeriod(size_t ch, uint16_t period_us)
{
  const uint16_t data = period_us & kThrottleMask;
  setData(ch, data);
}

inline bool PWM::transfer()
{
  if (!spi_.transfer(tx_, rx_, kSpiBufSize))
    return false;

  return true;
}

inline void PWM::setData(size_t ch, uint16_t data)
{
  tx_[ch * kChannelBytes] = data & 0xFF;    // Little byte
  tx_[ch * kChannelBytes + 1] = data >> 4;  // Big byte
}
}  // namespace a1
