#pragma once

#include <unistd.h>

#include <navio2/pwm.hpp>
#include <navio2/adc.hpp>

namespace tobas_calibration
{
/* 全てのPWMピンに対してキャリブレーションを行う． */
class EscCalibration
{
public:
  explicit EscCalibration();

  void run();

private:
  navio::PWM pwm_;
  navio::ADC adc_;

  void waitForBatteryDisconnected();
  void sendMaximum();
  void sendMinimum();

  void setPeriod(const double& period);
  void setPeriodAndSleep(const double& period);
  bool isBatteryConnected();
};
}  // namespace tobas_calibration
