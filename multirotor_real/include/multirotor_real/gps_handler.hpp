#pragma once

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <Common/Ublox.h>
#include <Common/Util.h>

class GpsHandler
{
public:
  GpsHandler(ros::NodeHandle& nh);

private:
  Ublox gps_;
  std::vector<double> gps_data_;
  sensor_msgs::NavSatFix gps_msg_;

  ros::Publisher gps_pub_;

  ros::Timer timer_;

  void timerCb(const ros::TimerEvent&);
};
