#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../../include/tobas_real/esc_calibration.hpp"
#include "../../include/tobas_real/common.hpp"

#define SLEEP_TIME_HIGH 3  // [s]
#define SLEEP_TIME_LOW 4   // [s]
#define INTERVAL 0.1       // [s]

using namespace std;

namespace tobas_real
{
EscCalibrator::EscCalibrator()
{
  if (getuid())
  {
    rosthrow("Not root.");
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
  rosInfo("Calibration finished.");
}

void EscCalibrator::setHigh()
{
  const ros::Time start_time = ros::Time::now();
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
    ros::Duration(INTERVAL).sleep();
  }
}

void EscCalibrator::setLow()
{
  const ros::Time start_time = ros::Time::now();
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
    ros::Duration(INTERVAL).sleep();
  }
}
}  // namespace tobas_real
