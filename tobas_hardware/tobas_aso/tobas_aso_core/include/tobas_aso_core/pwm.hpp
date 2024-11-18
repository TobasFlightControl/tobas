#pragma once

#include <cstddef>
#include <iostream>

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
  static constexpr uint32_t kSpiClockFreq = 27'000'000;                // [Hz] F722のSPI2の最大値
  static constexpr uint16_t kThrottleMask = (1 << 11) - 1;

public:
  explicit PWM();

  bool initialize();

  bool setPeriod(size_t ch, uint16_t period_us);
  bool transfer();

private:
  linux::SPIdev spi_;

  bool setData(size_t ch, uint16_t data);
};
}  // namespace aso
