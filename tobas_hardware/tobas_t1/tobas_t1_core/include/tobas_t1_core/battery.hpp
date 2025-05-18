#pragma once

#include <cstddef>

#include <tobas_linux/spi_dev.hpp>

namespace t1
{
class Battery
{
  static constexpr char kSpiDevice[] = "/dev/spidev0.1";
  static constexpr uint32_t kSPIClockFreq = 30'000'000;  // [Hz]
  static constexpr size_t kChannelSize = 2;
  static constexpr size_t kSPIBufSize = kChannelSize;

public:
  explicit Battery();

  bool initialize();
  bool read(float& voltage, float& current);

private:
  linux::SPIdev spi_;
  uint32_t tx_buf_[kSPIBufSize];
  uint32_t rx_buf_[kSPIBufSize];
};
}  // namespace t1
