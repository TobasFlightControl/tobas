#include <stdexcept>
#include <cassert>

#include "../include/tobas_navio_ros/common.hpp"

using namespace std;

namespace tobas_navio_ros
{
void setupRCOutput(navio::PWM& pwm, const size_t& channel)
{
  if (!pwm.initialize(channel))
    throw runtime_error("Failed to initialize RC output on CH" + to_string(channel) + ".");
  if (!pwm.setFrequency(channel, kPwmFrequency))
    throw runtime_error("Failed to set PWM frequency on CH" + to_string(channel) + ".");
  if (!pwm.enable(channel))
    throw runtime_error("Failed to enable RC output on CH" + to_string(channel) + ".");
}
}  // namespace tobas_navio_ros
