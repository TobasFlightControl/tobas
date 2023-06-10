#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
void setupRCOutput(RCOutput_Navio2& pwm, uint32_t channel)
{
  const auto pin = pinFromChannel(channel);

  if (!pwm.initialize(channel))
  {
    throw dh_ros::RuntimeError("Failed to initialize RC output on PIN" + to_string(pin) + ".");
  }

  if (!pwm.set_frequency(channel, kPwmFrequency))
  {
    throw dh_ros::RuntimeError("Failed to set PWM frequency on PIN" + to_string(pin) + ".");
  }

  if (!pwm.enable(channel))
  {
    throw dh_ros::RuntimeError("RC output on PIN" + to_string(pin) + " is disabled.");
  }

  rosInfo("Setup for RC output on PIN" << pin << " finished successfully.");
}

uint32_t channelFromPin(uint32_t pin)
{
  assert(1 <= pin && pin <= kServoRailSize);
  return pin - 1;
}

uint32_t pinFromChannel(uint32_t channel)
{
  assert(channel < kServoRailSize);
  return channel + 1;
}
}  // namespace tobas_real
