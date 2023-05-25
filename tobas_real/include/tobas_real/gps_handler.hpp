#pragma once

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <Common/Ublox.h>

#include <dh_ros_tools/node.hpp>
#include <dh_ros_tools/timer.hpp>

#include <tobas_msgs/LinearVelocityWithCovariance.h>

namespace tobas_real
{
class GpsHandler : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

  using GpsMsg = sensor_msgs::NavSatFix;
  using VelMsg = tobas_msgs::LinearVelocityWithCovariance;

public:
  explicit GpsHandler();

private:
  Ublox gps_;
  std::vector<double> data_;
  GpsMsg gps_msg_;
  VelMsg vel_msg_;
  bool cov_received_;

  // Publisher
  ros::Publisher gps_pub_;
  ros::Publisher vel_pub_;

  // Timer
  dh_ros::Timer main_loop_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void mainLoopTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_real
