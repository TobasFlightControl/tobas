#pragma once

#include <ros/ros.h>
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
  ros::NodeHandle nh_;
  RCOutput_Navio2 pwm_;

  void setHigh();
  void setLow();
};
}  // namespace tobas_real
