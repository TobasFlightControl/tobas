#include <iostream>

#include <tobas_math/core.hpp>

#include "../include/tobas_a1_core/dshot.hpp"
#include "../include/tobas_a1_core/constants.hpp"

using namespace std;

namespace a1
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

bool DShot::setThrottle(size_t ch, uint16_t throttle, bool telem)
{
  // Check throttle range
  if (throttle > kMaxThrottle)
  {
    cerr << "DSHOT thrrotle out of range." << endl;
    return false;
  }

  // Create DSHOT protocol data
  uint16_t data;

  // Throttle
  data = throttle & kThrottleMask;

  // Telemetry
  data = (data << 1) + static_cast<uint8_t>(telem);

  // CRC
  const uint8_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F;
  data = (data << 4) + crc;

  // Set DSHOT protocol data
  return setData(ch, data);
}

bool DShot::setDisabled(size_t ch)
{
  return setThrottle(ch, kDShotDisableCommand, false);
}

bool DShot::transfer()
{
  if (!spi_.transfer(kSpiBufSize))
    return false;

  return true;
}

bool DShot::setData(size_t ch, uint16_t data)
{
  if (ch >= kChannelSize)
  {
    cerr << "DSHOT channel out of range." << endl;
    return false;
  }

  spi_.tx[ch * kChannelBytes] = data & 0xFF;    // Little byte
  spi_.tx[ch * kChannelBytes + 1] = data >> 4;  // Big byte

  return true;
}
}  // namespace a1
