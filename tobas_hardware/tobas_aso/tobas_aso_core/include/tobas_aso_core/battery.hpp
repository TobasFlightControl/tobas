#pragma once

#include <cstddef>

#include <tobas_linux/spi_dev.hpp>

namespace aso
{
class Battery
{
private:
  static constexpr size_t kSPIBufSize = 2;
  static constexpr uint32_t kSPIClockFreq = 30'000'000;  // [Hz]

public:
  explicit Battery();

  bool initialize();
  bool read(double& voltage, double& current);

  inline const double& getVoltage() const;
  inline const double& getCurrent() const;

private:
  linux::SPIdev spi_;
  uint32_t tx_buf_[kSPIBufSize];
  uint32_t rx_buf_[kSPIBufSize];
};
}  // namespace aso
