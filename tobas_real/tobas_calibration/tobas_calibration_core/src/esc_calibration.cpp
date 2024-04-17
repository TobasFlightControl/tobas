#include <iostream>
#include <chrono>
#include <unistd.h>

#include <tobas_std_tools/time.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_navio_ros/common.hpp>

#include "../include/tobas_calibration_core/esc_calibration.hpp"

#define SLEEP_HIGH 3.   // [s]
#define SLEEP_LOW 5.    // [s]
#define INTERVAL 10000  // [us]
#define A2_VALUE_THRESHOLD 300

using namespace std;
using namespace chrono;

namespace tobas_calibration
{
EscCalibration::EscCalibration()
{
  for (size_t channel = 0; channel < tobas::kServoRailSize; ++channel)
    tobas_navio_ros::setupRCOutput(pwm_, channel);

  if (adc_.initialize() < 0)
    throw runtime_error("Failed to initialize ADC driver.");
}

void EscCalibration::run()
{
  PRINT_WARN("Please make sure propellers are memoved from motors.");

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
    usleep(INTERVAL);

  cout << "Battery disconnection detected." << endl;
}

void EscCalibration::sendMaximum()
{
  cout << "Please connect battery." << endl;

  while (!isBatteryConnected())
    setPeriodAndSleep(tobas_navio_ros::kPwmMax);

  cout << "Battery connection detected." << endl;
  cout << "Sending maximum throttle." << endl;

  const auto start_time = system_clock::now();
  while (duration<double>(system_clock::now() - start_time).count() < SLEEP_HIGH)
    setPeriodAndSleep(tobas_navio_ros::kPwmMax);
}

void EscCalibration::sendMinimum()
{
  cout << "Sending minimum throttle." << endl;

  const auto start_time = system_clock::now();
  while (duration<double>(system_clock::now() - start_time).count() < SLEEP_LOW)
    setPeriodAndSleep(tobas_navio_ros::kPwmMin);
}

void EscCalibration::setPeriod(const double& period)
{
  for (size_t channel = 0; channel < tobas::kServoRailSize; ++channel)
    if (!pwm_.setDutyCycle(channel, period))
      throw runtime_error("Failed to set PWM duty cycle.");
}

void EscCalibration::setPeriodAndSleep(const double& period)
{
  setPeriod(period);
  usleep(INTERVAL);
}

bool EscCalibration::isBatteryConnected()
{
  const auto a2_value = adc_.read(tobas_navio_ros::kPowerModuleVoltageChannel);
  return a2_value > A2_VALUE_THRESHOLD;
}
}  // namespace tobas_calibration
