#pragma once

#include <Navio2/RCOutput_Navio2.h>

namespace tobas_real
{
/* 全てのPWMピンに対してキャリブレーションを行う． */
class EscCalibrator
{
  static constexpr uint32_t kSleepHigh = 8000000;  // [us]
  static constexpr uint32_t kSleepLow = 4000000;   // [us]
  static constexpr uint32_t kInterval = 100000;    // [us]

public:
  explicit EscCalibrator();

  void run();

private:
  RCOutput_Navio2 pwm_;

  void setHigh();
  void setLow();
};
}  // namespace tobas_real
