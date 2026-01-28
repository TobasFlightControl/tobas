#pragma once

#include <cstddef>

#include <tobas_linux/spi_dev.hpp>

namespace fc1xx
{
class Battery
{
  static constexpr char kSpiDevice[] = "/dev/spidev0.1";
  static constexpr uint32_t kSpiClockFreq = 30'000'000;  // [Hz]
  static constexpr size_t kChannelSize = 2;

public:
  explicit Battery();

  bool initialize();
  bool read(float& voltage, float& current);

private:
  linux::SPIdev spi_;
  uint32_t tx_buf_[kChannelSize];
  uint32_t rx_buf_[kChannelSize];
};
}  // namespace fc1xx
