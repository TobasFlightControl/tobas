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
}  // namespace a1
