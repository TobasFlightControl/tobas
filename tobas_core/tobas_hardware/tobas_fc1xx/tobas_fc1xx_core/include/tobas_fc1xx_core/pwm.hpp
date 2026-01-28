#pragma once

#include <tobas_algorithm/crc.hpp>
#include <tobas_linux/spi_dev.hpp>

namespace fc1xx
{
class PWM
{
public:
  static constexpr size_t kChannelSize = 8;

private:
  static constexpr char kSpiDevice[] = "/dev/spidev1.1";
  static constexpr uint32_t kSpiClockFreq = 50'000'000;  // [Hz]
  static constexpr uint16_t kMaxPeriod = 2500;           // [us]

public:
  explicit PWM();

  bool initialize();

  bool setPeriod(size_t ch, uint16_t period_us);
  bool transfer();

private:
  linux::SPIdev spi_;
  uint16_t tx_buf_[kChannelSize + 2] = {};
  uint16_t rx_buf_[kChannelSize + 2] = {};

  algo::CRC32Left crc_;
};
}  // namespace fc1xx
