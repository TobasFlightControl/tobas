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
  if (!spi_.initialize(spi_device::kBatteryDev, tx_buf_, rx_buf_, kSPIClockFreq))
    return false;

  return true;
}

bool Battery::read(double& voltage, double& current)
{
  if (!spi_.transfer(sizeof(tx_buf_)))
    return false;

  voltage = static_cast<double>(rx_buf_[0]) * 1e-6;
  current = static_cast<double>(static_cast<int32_t>(rx_buf_[1])) * 1e-6;

  return true;
}
}  // namespace aso
