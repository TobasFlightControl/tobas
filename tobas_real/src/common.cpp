#include <dh_ros_tools/exception.hpp>

#include "../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
void setupRCOutput(RCOutput_Navio2& pwm, uint32_t channel)
{
  const auto pin = pinFromChannel(channel);

  if (!pwm.initialize(channel))
  {
    rosthrow("Failed to initialize RC output on PIN" << pin << ".");
  }

  if (!pwm.set_frequency(channel, kPwmFrequency))
  {
    rosthrow("Failed to set PWM frequency on PIN" << pin << ".");
  }

  if (!pwm.enable(channel))
  {
    rosthrow("RC output on PIN" << pin << " is disabled.");
  }
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
