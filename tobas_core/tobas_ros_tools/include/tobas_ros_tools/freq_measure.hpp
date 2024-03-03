#pragma once

#include <ros/ros.h>
#include <string>

namespace tobas_ros
{
class FreqMeasure
{
public:
  FreqMeasure(
    const std::string& name,
    const double& warn_period = 3.,
    const double& warn_rate = 0.9);

  void setFreq(const double& freq);

  void count();

private:
  const std::string name_;
  const double warn_period_;
  const double warn_rate_;
  double freq_;
  int cnt_ideal_;
  int cnt_th_;
  int cnt_;
  bool ready_;
  bool first_sleep_;
  ros::Time t_last_loop_;
};
}  // namespace tobas_ros
