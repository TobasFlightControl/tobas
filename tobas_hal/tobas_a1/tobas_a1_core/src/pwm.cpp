#include "../include/tobas_a1_core/pwm.hpp"
#include "../include/tobas_a1_core/constants.hpp"

using namespace std;

namespace a1
{
PWM::PWM()
{
}

bool PWM::initialize()
{
  if (!spi_.initialize(spi_device::kDshotDev, kSpiClockSpeed))
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
  if (!spi_.transfer(tx_, rx_, kSpiBufSize))
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

  tx_[ch * kChannelBytes] = data & 0xFF;    // Little byte
  tx_[ch * kChannelBytes + 1] = data >> 4;  // Big byte

  return true;
}
}  // namespace a1
