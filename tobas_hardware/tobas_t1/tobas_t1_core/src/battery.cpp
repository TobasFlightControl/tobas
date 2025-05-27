#include "tobas_t1_core/battery.hpp"

#include <bit>

namespace t1
{
Battery::Battery()
{
}

bool Battery::initialize()
{
  if (!spi_.initialize(kSpiDevice, tx_buf_, rx_buf_, kSPIClockFreq)) {
    return false;
  }

  return true;
}

bool Battery::read(float& voltage, float& current)
{
  if (!spi_.transfer(sizeof(tx_buf_))) {
    return false;
  }

  voltage = std::bit_cast<float>(rx_buf_[0]);
  current = std::bit_cast<float>(rx_buf_[1]);

  return true;
}
}  // namespace t1
