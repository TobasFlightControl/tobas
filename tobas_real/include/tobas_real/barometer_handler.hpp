#pragma once

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include <Common/MS5611.h>

class BarometerHandler
{
  using BarMsg = sensor_msgs::FluidPressure;

public:
  BarometerHandler();

private:
  ros::NodeHandle nh_;

  MS5611 barometer_;
  BarMsg bar_msg_;

  // roaparam
  std::string drone_name_;

  // Publisher
  ros::Publisher bar_pub_;

  ros::Timer timer_;

  void getRosParams();
  void registerPublishers();
  void timerCb(const ros::TimerEvent&);
};
