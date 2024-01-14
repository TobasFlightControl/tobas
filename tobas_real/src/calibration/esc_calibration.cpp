#include <iostream>
#include <chrono>
#include <unistd.h>

#include <tobas_std_tools/time.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_std_tools/unix.hpp>

#include "../../include/tobas_real/calibration/esc_calibration.hpp"
#include "../../include/tobas_real/common.hpp"

using namespace std;
using namespace chrono;

namespace tobas_real
{
EscCalibration::EscCalibration()
{
  if (!tobas_std::isSuperUser())
    throw runtime_error("Please execute with root privileges.");

  for (size_t channel = 0; channel < kServoRailSize; ++channel)
    setupRCOutput(pwm_, channel);

  adc_.initialize();
}

void EscCalibration::run()
{
  TOBAS_WARN("Please make sure propellers are memoved from motors.");

  cout << "Please press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');

  if (isBatteryConnected())
  {
    cout << "Please disconnect battery." << endl;
    waitForBatteryDisconnected();
  }

  sendMaximum();
  sendMinimum();

  cout << "Calibration finished." << endl;
}

void EscCalibration::waitForBatteryDisconnected()
{
  while (isBatteryConnected())
    usleep(kInterval);

  cout << "Battery disconnection detected." << endl;
}

void EscCalibration::sendMaximum()
{
  cout << "Please connect battery." << endl;

  while (!isBatteryConnected())
    setPeriodAndSleep(kPwmMax);

  cout << "Battery connection detected." << endl;
  cout << "Sending maximum throttle." << endl;

  const auto start_time = system_clock::now();
  while (duration_cast<microseconds>(system_clock::now() - start_time).count() < kSleepHigh)
    setPeriodAndSleep(kPwmMax);
}

void EscCalibration::sendMinimum()
{
  cout << "Sending minimum throttle." << endl;

  const auto start_time = system_clock::now();
  while (duration_cast<microseconds>(system_clock::now() - start_time).count() < kSleepLow)
    setPeriodAndSleep(kPwmMin);
}

void EscCalibration::setPeriod(const double& period)
{
  for (size_t channel = 0; channel < kServoRailSize; ++channel)
    if (!pwm_.setDutyCycle(channel, period))
      throw runtime_error("Failed to set PWM duty cycle.");
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
