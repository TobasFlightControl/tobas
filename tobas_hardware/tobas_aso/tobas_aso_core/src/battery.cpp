#include <iostream>

#include "../include/tobas_aso_core/battery.hpp"
#include "../include/tobas_aso_core/constants.hpp"

using namespace std;

namespace aso
{
Battery::Battery()
{
}

bool Battery::initialize()
{
  if (!spi_.initialize(spi_device::kBatteryDev, kSpiClockFreq))
    return false;

  return true;
}

bool Battery::read(double& voltage, double& current)
{
  if (!spi_.transfer(8))
    return false;

  voltage = static_cast<double>(*(uint32_t*)spi_.rx) * 1e-6;
  current = static_cast<double>(*((int32_t*)spi_.rx + 1));

  return true;
}
}  // namespace aso
