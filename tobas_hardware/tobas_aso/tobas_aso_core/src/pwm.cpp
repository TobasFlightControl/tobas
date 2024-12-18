#include <iostream>

#include "../include/tobas_aso_core/pwm.hpp"
#include "../include/tobas_aso_core/constants.hpp"

using namespace std;

namespace aso
{
PWM::PWM()
{
}

bool PWM::initialize()
{
  if (!spi_.initialize(spi_device::kPwmDev, kSpiClockFreq))
    return false;

  return true;
}

bool PWM::setPeriod(size_t ch, uint16_t period_us)
{
  const uint16_t data = period_us & kThrottleMask;
  return setData(ch, data);
}

bool PWM::transfer()
{
  if (!spi_.transfer(kSpiBufSize))
    return false;

  return true;
}

bool PWM::setData(size_t ch, uint16_t data)
{
  if (ch >= kChannelSize)
  {
    cerr << "PWM channel out of range." << endl;
    return false;
  }

  spi_.tx[ch * kChannelBytes] = data & 0xFF;    // Little byte
  spi_.tx[ch * kChannelBytes + 1] = data >> 8;  // Big byte

  return true;
}
}  // namespace aso
