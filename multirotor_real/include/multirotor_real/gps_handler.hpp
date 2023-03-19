#pragma once

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <Common/Ublox.h>

#include <multirotor_msgs/LinearVelocityWithCovarianceStamped.h>

class GpsHandler
{
  using GpsMsg = sensor_msgs::NavSatFix;
  using VelMsg = multirotor_msgs::LinearVelocityWithCovarianceStamped;

public:
  GpsHandler(ros::NodeHandle& nh);

private:
  Ublox gps_;
  std::vector<double> data_;
  GpsMsg gps_msg_;
  VelMsg vel_msg_;
  bool cov_received_;

  ros::Publisher gps_pub_;
  ros::Publisher vel_pub_;

  ros::Timer timer_;

  void timerCb(const ros::TimerEvent&);
};
