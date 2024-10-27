#include <iostream>

#include "../include/tobas_aso_core/dshot.hpp"
#include "../include/tobas_aso_core/constants.hpp"

using namespace std;

namespace aso
{
DShot::DShot()
{
}

bool DShot::initialize()
{
  if (!spi_.initialize(spi_device::kDshotDev, kSpiClockFreq, kSpiBufSize))
    return false;

  for (size_t ch = 0; ch < kChannelSize; ++ch)
    setDisabled(ch);

  return true;
}

bool DShot::setThrottle(size_t ch, uint16_t throttle)
{
  if (ch >= kChannelSize)
  {
    cerr << "DSHOT channel out of range." << endl;
    return false;
  }

  if (throttle > kMaxThrot)
  {
    cerr << "DSHOT thrrotle out of range." << endl;
    return false;
  }

  spi_.tx[ch * kChannelBytes] = throttle & 0xFF;    // Little byte
  spi_.tx[ch * kChannelBytes + 1] = throttle >> 8;  // Big byte

  return true;
}

bool DShot::setDisabled(size_t ch)
{
  return setThrottle(ch, kDShotDisableCommand);
}

bool DShot::transfer()
{
  if (!spi_.transfer(kSpiBufSize))
    return false;

  return true;
}
}  // namespace aso
