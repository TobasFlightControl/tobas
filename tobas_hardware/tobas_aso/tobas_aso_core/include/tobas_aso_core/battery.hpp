#pragma once

#include <cstddef>

#include <tobas_linux/spi_dev.hpp>

namespace aso
{
class Battery
{
private:
  static constexpr uint32_t kSpiClockFreq = 45'000'000;  // [Hz] STM32H7A3RITxのTransmit Onlyの最大値

public:
  explicit Battery();

  bool initialize();
  bool read(double& voltage, double& current);

  inline const double& getVoltage() const;
  inline const double& getCurrent() const;

private:
  linux::SPIdev spi_;
};
}  // namespace aso
