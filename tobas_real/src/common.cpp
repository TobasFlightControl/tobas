#include <dh_ros_tools/exception.hpp>

#include "../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
void setupRCOutput(RCOutput_Navio2& pwm, const uint32_t& channel)
{
  const auto pin = pinFromChannel(channel);

  if (!pwm.initialize(channel))
  {
    throw runtime_error("Failed to initialize RC output on PIN" + to_string(pin) + ".");
  }

  if (!pwm.set_frequency(channel, kPwmFrequency))
  {
    throw runtime_error("Failed to set PWM frequency on PIN" + to_string(pin) + ".");
  }

  if (!pwm.enable(channel))
  {
    throw runtime_error("RC output on PIN" + to_string(pin) + " is disabled.");
  }
}

uint32_t channelFromPin(const uint32_t& pin)
{
  assert(1 <= pin && pin <= kServoRailSize);
  return pin - 1;
}

uint32_t pinFromChannel(const uint32_t& channel)
{
  assert(channel < kServoRailSize);
  return channel + 1;
}
}  // namespace tobas_real
