#pragma once

#include <tobas_algorithm/crc.hpp>
#include <tobas_linux/spi_dev.hpp>

namespace aso
{
class PWM
{
public:
  static constexpr size_t kChannelSize = 8;
  static constexpr size_t kSPIBufSize = kChannelSize + 2;  // Data + CRC32

private:
  static constexpr size_t kChannelBytes = 2;                           // 1チャネルあたりのバイト数
  static constexpr size_t kSpiBufSize = kChannelSize * kChannelBytes;  // SPIバッファのサイズ
  static constexpr uint32_t kSPIClockFreq = 50'000'000;                // [Hz]
  static constexpr uint16_t kMaxPeriod = 2500;                         // [us]

public:
  explicit PWM();

  bool initialize();

  bool setPeriod(size_t ch, uint16_t period_us);
  bool transfer();

private:
  linux::SPIdev spi_;
  uint16_t tx_buf_[kSPIBufSize];
  uint16_t rx_buf_[kSPIBufSize];

  algo::CRC32Left crc_;
};
}  // namespace aso
