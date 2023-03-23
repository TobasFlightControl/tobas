#pragma once

#include <geometry_msgs/Point.h>

#include <multirotor_msgs/LinearVelocity.h>
#include <multirotor_msgs/LinearAccel.h>

class PositionController
{
public:
  PositionController();

  void update(
    const geometry_msgs::Point& pos,
    const geometry_msgs::Point& pos_des,
    const geometry_msgs::Vector3& vel,
    const geometry_msgs::Vector3& vel_des,
    multirotor_msgs::LinearAccel& acc_out);

  void reconfigure(double natural_freq, double damp_ratio);

private:
  double kp_;
  double kd_;
};
