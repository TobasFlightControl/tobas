#include <ros/ros.h>

#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include <multirotor_tools/operators.hpp>
#include <multirotor_tools/conversions/msg_msg.hpp>

#include "../../include/multirotor_controller/position_controller.hpp"

PositionController::PositionController()
{
  double natural_freq = dh_ros::getParam<double>("~natural_frequency");
  double damp_ratio = dh_ros::getParam<double>("~damping_ratio");
  reconfigure(natural_freq, damp_ratio);
}

void PositionController::update(
  const geometry_msgs::Point& pos,
  const geometry_msgs::Point& pos_des,
  const multirotor_msgs::LinearVelocity& vel,
  const multirotor_msgs::LinearVelocity& vel_des,
  multirotor_msgs::LinearAccel& acc_out)
{
  const auto& acc_out_tmp = kp_ * (pos_des - pos) + kd_ * (vel_des - vel);
  tf::Vector3ToLinearAccel(acc_out_tmp, acc_out);
}

void PositionController::reconfigure(double natural_freq, double damp_ratio)
{
  ROS_ASSERT(natural_freq > 0.);
  ROS_ASSERT(damp_ratio > 0.);

  kp_ = dh_std::sqr(natural_freq);
  kd_ = 2. * damp_ratio * natural_freq;
}
