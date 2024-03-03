#pragma once

#include <unistd.h>

#include <Navio2/PWM.h>
#include <Navio2/ADC_Navio2.h>

namespace tobas_real
{
/* 全てのPWMピンに対してキャリブレーションを行う． */
class EscCalibration
{
public:
  explicit EscCalibration();

  void run();

private:
  PWM pwm_;
  ADC_Navio2 adc_;

  void waitForBatteryDisconnected();
  void sendMaximum();
  void sendMinimum();

  void setPeriod(const double& period);
  void setPeriodAndSleep(const double& period);
  bool isBatteryConnected();
};
}  // namespace tobas_real
