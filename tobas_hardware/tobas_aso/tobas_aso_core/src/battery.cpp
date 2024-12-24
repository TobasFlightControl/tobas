#include <iostream>

#include "../include/tobas_aso_core/battery.hpp"
#include "../include/tobas_aso_core/constants.hpp"

using namespace std;

namespace aso
{
Battery::Battery() : crc_(algo::CRC32Left::CRC_32)
{
  crc_.initialize();
}

bool Battery::initialize()
{
  if (!spi_.initialize(spi_device::kBatteryDev, tx_buf_, rx_buf_, kSPIClockFreq))
    return false;

  return true;
}

bool Battery::read(double& voltage, double& current)
{
  // Transfer
  if (!spi_.transfer(sizeof(tx_buf_)))
    return false;

  // Check CRC
  // const auto cs = rx_buf_[kChannelSize];
  // const auto cr = crc_.compute((uint8_t*)rx_buf_, sizeof(uint32_t) * kChannelSize);
  // if (cs != cr)
  // {
  //   cerr << "CRC failed: " << cs << " != " << cr << endl;
  //   return false;
  // }

  voltage = static_cast<double>(rx_buf_[0]) * 1e-6;
  current = static_cast<double>(static_cast<int32_t>(rx_buf_[1])) * 1e-6;

  return true;
}
}  // namespace aso
