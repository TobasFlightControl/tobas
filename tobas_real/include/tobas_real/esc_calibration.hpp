#pragma once

#include <Navio2/RCOutput_Navio2.h>

namespace tobas_real
{
/* 全てのPWMピンに対してキャリブレーションを行う． */
class EscCalibrator
{
public:
  explicit EscCalibrator();

  void run();

private:
  void setHigh();
  void setLow();

  RCOutput_Navio2 pwm_;
};
}  // namespace tobas_real
