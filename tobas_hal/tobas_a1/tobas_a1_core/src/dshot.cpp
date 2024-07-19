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
  if (!spi_.initialize(spi_device::kDshotDev, kSpiClockSpeed))
    return false;

  for (size_t ch = 0; ch < kChannelSize; ++ch)
    setThrottle(ch, kDShotDisableThrottle);

  return true;
}
}  // namespace a1
