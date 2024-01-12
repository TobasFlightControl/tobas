#include <stdexcept>
#include <cassert>

#include "../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
void setupRCOutput(RCOutput_Navio2& pwm, const size_t& channel)
{
  const auto pin = pinFromChannel(channel);

  if (!pwm.initialize(channel))
    throw runtime_error("Failed to initialize RC output on PIN" + to_string(pin) + ".");
  if (!pwm.setFrequency(channel, kPwmFrequency))
    throw runtime_error("Failed to set PWM frequency on PIN" + to_string(pin) + ".");
  if (!pwm.enable(channel))
    throw runtime_error("RC output on PIN" + to_string(pin) + " is disabled.");
}

size_t channelFromPin(const size_t& pin)
{
  assert(1 <= pin && pin <= kServoRailSize);
  return pin - 1;
}

size_t pinFromChannel(const size_t& channel)
{
  assert(channel < kServoRailSize);
  return channel + 1;
}
}  // namespace tobas_real
