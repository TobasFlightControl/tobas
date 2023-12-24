#include <string>
#include <iostream>

#include "../../include/Common/Util.h"
#include "../../include/Navio2/PWM.h"

using namespace std;

PWM::PWM()
{
}

bool PWM::init(const size_t& channel)
{
  const auto err = write_file("/sys/class/pwm/pwmchip0/export", "%u", channel);
  return err >= 0 || err == -EBUSY;
}

bool PWM::enable(const size_t& channel)
{
  const string path = "/sys/class/pwm/pwmchip0/pwm" + to_string(channel) + "/enable";
  return write_file(path.c_str(), "1") >= 0;
}

bool PWM::setPeriod(const size_t& channel, const size_t& freq)
{
  const string path = "/sys/class/pwm/pwmchip0/pwm" + to_string(channel) + "/period";
  const int period_ns = 1e+9 / freq;
  return write_file(path.c_str(), "%u", period_ns) >= 0;
}

bool PWM::setDutyCycle(const size_t& channel, const double& period_ms)
{
  const string path = "/sys/class/pwm/pwmchip0/pwm" + to_string(channel) + "/duty_cycle";
  const int period_ns = period_ms * 1e+6;
  return write_file(path.c_str(), "%u", period_ns) >= 0;
}
