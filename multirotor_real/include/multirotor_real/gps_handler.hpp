#pragma once

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <Common/Ublox.h>

class GpsHandler
{
  using GpsMsg = sensor_msgs::NavSatFix;

public:
  GpsHandler(ros::NodeHandle& nh);

private:
  Ublox gps_;
  std::vector<double> gps_data_;
  GpsMsg gps_msg_;

  ros::Publisher gps_pub_;

  ros::Timer timer_;

  void timerCb(const ros::TimerEvent&);
};
