#include <string>
#include <iostream>
#include <unistd.h>

#include "../include/navio2/util.hpp"
#include "../include/navio2/pwm.hpp"

#define NON_ROOT_SLEEP 100000  // [us]

using namespace std;

namespace navio
{
PWM::PWM()
{
}

bool PWM::initialize(const size_t& channel)
{
  const auto err = write_file("/sys/class/pwm/pwmchip0/export", "%u", channel);

  // 非rootの場合は，udevによってPWMデバイスがシステムに追加された際にアクセス権の変更等の遅延が生じるため，少し待つ
  if (getuid() != 0)
    usleep(NON_ROOT_SLEEP);

  return err >= 0 || err == -EBUSY;
}

bool PWM::remove(const size_t& channel)
{
  const auto err = write_file("/sys/class/pwm/pwmchip0/unexport", "%u", channel);
  return err >= 0;
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
}  // namespace navio
