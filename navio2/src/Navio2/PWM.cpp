#include <string>
#include <iostream>

#include "../../include/Common/Util.h"
#include "../../include/Navio2/PWM.h"

using namespace std;

PWM::PWM()
{
}

bool PWM::initialize(const size_t& channel)
{
  const auto err = write_file("/sys/class/pwm/pwmchip0/export", "%u", channel);
  return err >= 0 || err == -EBUSY;
}

bool PWM::enable(const size_t& channel)
{
  const string path = "/sys/class/pwm/pwmchip0/pwm" + to_string(channel) + "/enable";
  return write_file(path.c_str(), "1") >= 0;
}

bool PWM::disable(const size_t& channel)
{
  const string path = "/sys/class/pwm/pwmchip0/pwm" + to_string(channel) + "/enable";
  return write_file(path.c_str(), "0") >= 0;
}

bool PWM::setFrequency(const size_t& channel, const size_t& freq)
{
  const string path = "/sys/class/pwm/pwmchip0/pwm" + to_string(channel) + "/period";
  const auto period_ns = static_cast<int>(1e+9 / freq);
  return write_file(path.c_str(), "%u", period_ns) >= 0;
}

bool PWM::setDutyCycle(const size_t& channel, const double& period_us)
{
  const string path = "/sys/class/pwm/pwmchip0/pwm" + to_string(channel) + "/duty_cycle";
  const auto period_ns = static_cast<int>(period_us * 1e+3);
  return write_file(path.c_str(), "%u", period_ns) >= 0;
}
