#pragma once

#include <ros/ros.h>

namespace tobas_ros
{
class Rate : public ros::Rate
{
  using super = ros::Rate;

public:
  Rate(const double& freq, const double& warn_period = 3., const double& warn_rate = 0.9);

  void sleep();

private:
  const double freq_;
  const double warn_period_;
  const int cnt_ideal_;
  const int cnt_th_;
  int cnt_;
  bool first_sleep_;
  ros::Time t_last_loop_;
};
}  // namespace tobas_ros
