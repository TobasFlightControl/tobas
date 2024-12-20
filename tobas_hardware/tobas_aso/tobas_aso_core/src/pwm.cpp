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
  if (ch >= kChannelSize)
  {
    cerr << "PWM channel out of range." << endl;
    return false;
  }

  if (period_us >= (1 << 11))
  {
    cerr << "PWM period must be lower than " << (1 << 11) << ".";
    return false;
  }

  *((uint16_t*)spi_.tx + ch) = period_us;
  return true;
}

bool PWM::transfer()
{
  if (!spi_.transfer(kSpiBufSize))
    return false;

  return true;
}
}  // namespace aso
