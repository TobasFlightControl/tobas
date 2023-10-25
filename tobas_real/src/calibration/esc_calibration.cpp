#include <iostream>
#include <chrono>
#include <unistd.h>

#include "../../include/tobas_real/calibration/esc_calibration.hpp"
#include "../../include/tobas_real/common.hpp"

using namespace std;
using namespace chrono;

namespace tobas_real
{
EscCalibrator::EscCalibrator()
{
  if (getuid())
  {
    throw runtime_error("Not root.");
  }

  for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
  {
    setupRCOutput(pwm_, channel);
  }
}

void EscCalibrator::run()
{
  setHigh();
  setLow();
  cout << "Calibration finished." << endl;
}

void EscCalibrator::setHigh()
{
  const auto start_time = system_clock::now();
  cout << "Send maximum throttle command for " << (kSleepHigh / 1000000) << " seconds." << endl;

  while (duration_cast<microseconds>(system_clock::now() - start_time).count() < kSleepHigh)
  {
    for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
    {
      if (!pwm_.setDutyCycle(channel, kPwmMax))
      {
        throw runtime_error("Failed to set high duty cycle.");
      }
    }
    usleep(kInterval);
  }
}

void EscCalibrator::setLow()
{
  const auto start_time = system_clock::now();
  cout << "Send minimum throttle command for " << (kSleepLow / 1000000) << " seconds." << endl;

  while (duration_cast<microseconds>(system_clock::now() - start_time).count() < kSleepLow)
  {
    for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
    {
      if (!pwm_.setDutyCycle(channel, kPwmMin))
      {
        throw runtime_error("Failed to set low duty cycle.");
      }
    }
    usleep(kInterval);
  }
}
}  // namespace tobas_real
