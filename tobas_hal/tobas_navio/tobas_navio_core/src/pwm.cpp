#include <unistd.h>

#include "../include/tobas_navio_core/pwm.hpp"

#define NON_ROOT_SLEEP 100000  // [us]

using namespace std;

namespace navio
{
PWM::PWM()
{
  for (size_t channel = 0; channel < kChannelCount; ++channel)
  {
    enable_paths_[channel] = "/sys/class/pwm/pwmchip0/pwm" + to_string(channel) + "/enable";
    period_paths_[channel] = "/sys/class/pwm/pwmchip0/pwm" + to_string(channel) + "/period";
    duty_paths_[channel] = "/sys/class/pwm/pwmchip0/pwm" + to_string(channel) + "/duty_cycle";
  }
}

bool PWM::initialize(const size_t& channel)
{
  const auto err = writeFile("/sys/class/pwm/pwmchip0/export", "%u", channel);

  // 非rootの場合は，udevによってPWMデバイスがシステムに追加された際にアクセス権の変更等の遅延が生じるため，少し待つ
  if (getuid() != 0)
    usleep(NON_ROOT_SLEEP);

  return err >= 0 || err == -EBUSY;
}

bool PWM::remove(const size_t& channel)
{
  return writeFile("/sys/class/pwm/pwmchip0/unexport", "%u", channel) >= 0;
}
}  // namespace navio
