#include <dh_ros_tools/console_message.hpp>

#include "../../include/tobas_real/esc_calibration.hpp"
#include "../../include/tobas_real/constants.hpp"

#define SLEEP_TIME_HIGH 3  // [s]
#define SLEEP_TIME_LOW 4   // [s]
#define INTERVAL 0.1       // [s]

using namespace std;

namespace tobas_real
{
EscCalibrator::EscCalibrator()
{
  for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
  {
    if (!(pwm_.initialize(channel)))
    {
      rosFatal("Failed to initialze PWM on channel " << channel << ".");
    }
    if (!pwm_.set_frequency(channel, kPwmFrequency))
    {
      rosFatal("Failed to set frequency on channel " << channel << ".");
    }
    if (!(pwm_.enable(channel)))
    {
      rosFatal("Failed to enable PWM on channel " << channel << ".");
    }
  }
}

void EscCalibrator::run()
{
  setHigh();
  setLow();
  rosInfo("Calibration finished.");
}

void EscCalibrator::setHigh()
{
  ros::Time start_time = ros::Time::now();
  rosInfo("Send maximum throttle command for " << SLEEP_TIME_HIGH << "seconds.");

  while ((ros::Time::now() - start_time).toSec() < SLEEP_TIME_HIGH)
  {
    for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
    {
      if (!pwm_.set_duty_cycle(channel, kPwmMax))
      {
        rosFatal("Failed to set high duty cycle on channel " << channel << ".");
      }
    }
    sleep(INTERVAL);
  }
}

void EscCalibrator::setLow()
{
  ros::Time start_time = ros::Time::now();
  rosInfo("Send minimum throttle command for " << SLEEP_TIME_LOW << "seconds.");

  while ((ros::Time::now() - start_time).toSec() < SLEEP_TIME_LOW)
  {
    for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
    {
      if (!pwm_.set_duty_cycle(channel, kPwmMin))
      {
        rosFatal("Failed to set low duty cycle on channel " << channel << ".");
      }
    }
    sleep(INTERVAL);
  }
}
}  // namespace tobas_real
