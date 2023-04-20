#pragma once

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

class PositionController
{
public:
  PositionController();

  void update(
    const geometry_msgs::Vector3& pos,
    const geometry_msgs::Vector3& pos_des,
    const geometry_msgs::Vector3& vel,
    const geometry_msgs::Vector3& vel_des,
    geometry_msgs::Vector3& acc_out);

  void reconfigure(double natural_freq, double damp_ratio);

private:
  double kp_;
  double kd_;
};
