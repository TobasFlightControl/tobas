#include <iostream>
#include <chrono>
#include <unistd.h>

#include <dh_std_tools/time.hpp>
#include <dh_std_tools/console.hpp>

#include "../../include/tobas_real/calibration/esc_calibration.hpp"
#include "../../include/tobas_real/common.hpp"

using namespace std;
using namespace chrono;

namespace tobas_real
{
EscCalibration::EscCalibration()
{
  if (getuid())
  {
    throw runtime_error("Not root.");
  }

  for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
  {
    setupRCOutput(pwm_, channel);
  }

  adc_.initialize();
}

void EscCalibration::run()
{
  DH_WARN("Please make sure propellers are memoved from motors and press Enter:");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');

  waitForBatteryDisconnected();
  sendMaximum();
  sendMinimum();

  cout << "Calibration finished." << endl;
}

void EscCalibration::waitForBatteryDisconnected()
{
  cout << "Please disconnect battery." << endl;

  while (isBatteryConnected())
  {
    sleep(kInterval);
  }

  cout << "Battery disconnection detected." << endl;
}

void EscCalibration::sendMaximum()
{
  cout << "Sending maximum throttle." << endl;
  cout << "Please connect battery." << endl;

  while (!isBatteryConnected())
  {
    setPeriodAndSleep(kPwmMax);
  }

  cout << "Battery connection detected." << endl;
  cout << "Wait for " << dh_std::secondsFromMicroSeconds(kSleepHigh) << "seconds." << endl;

  const auto start_time = system_clock::now();
  while (duration_cast<microseconds>(system_clock::now() - start_time).count() < kSleepHigh)
  {
    setPeriodAndSleep(kPwmMax);
  }

  cout << "Finished setting maximum value." << endl;
}

void EscCalibration::sendMinimum()
{
  cout << "Sending minimum throttle." << endl;
  cout << "Wait for " << dh_std::secondsFromMicroSeconds(kSleepLow) << "seconds." << endl;

  const auto start_time = system_clock::now();
  while (duration_cast<microseconds>(system_clock::now() - start_time).count() < kSleepLow)
  {
    setPeriodAndSleep(kPwmMin);
  }

  cout << "Finished setting minimum value." << endl;
}

void EscCalibration::setPeriod(const double& period)
{
  for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
  {
    if (!pwm_.setDutyCycle(channel, period))
      throw runtime_error("Failed to set PWM duty cycle.");
  }
}

void EscCalibration::setPeriodAndSleep(const double& period)
{
  setPeriod(period);
  usleep(kInterval);
}

bool EscCalibration::isBatteryConnected()
{
  const auto a2_value = adc_.read(kPowerModuleVoltageChannel);
  return a2_value > kA2ValueThreshold;
}
}  // namespace tobas_real
