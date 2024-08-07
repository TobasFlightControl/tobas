#include <tobas_math/core.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_a1_ros/pwm_driver.hpp"

using namespace std;

namespace a1
{
PWMDriver::PWMDriver(, const std::string& name) : super(node, pnh, name)
{
  if (!pwm_.initialize())
    TOBAS_EXIT("Failed to initialize PWM driver.");

  pwms_sub_ = createSubscriber(tobas::kPwmCmdTopic, &self::pwmsCb, this);
}

void PWMDriver::pwmsCb(const tobas_msgs::PwmArray::ConstSharedPtr& pwms)
{
  // Set PWM periods of each channel
  for (const auto& elem : pwms->pwms)
  {
    if (elem.channel >= PWM::kChannelSize)
    {
      TOBAS_ERROR("PWM channel ", elem.channel, " does not exist.");
      continue;
    }

    TOBAS_ASSERT(pwm_.setPeriod(elem.channel, elem.period));
  }

  // Send PWM pwms
  if (!pwm_.transfer())
    TOBAS_ERROR("Failed to send PWM command.");
}
}  // namespace a1
