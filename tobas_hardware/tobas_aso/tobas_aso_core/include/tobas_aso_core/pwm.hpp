#pragma once

#include <cstddef>

#include <tobas_linux/spi_dev.hpp>

namespace aso
{
class PWM
{
public:
  static constexpr size_t kChannelSize = 8;

private:
  static constexpr size_t kChannelBytes = 2;                           // 1チャネルあたりのバイト数
  static constexpr size_t kSpiBufSize = kChannelSize * kChannelBytes;  // SPIバッファのサイズ
  static constexpr uint32_t kSpiClockFreq = 50'000'000;                // [Hz]
  static constexpr uint16_t kThrottleMask = (1 << 11) - 1;

public:
  explicit PWM();

  bool initialize();

  bool setPeriod(size_t ch, uint16_t period_us);
  bool transfer();

private:
  linux::SPIdev spi_;
};
}  // namespace aso
