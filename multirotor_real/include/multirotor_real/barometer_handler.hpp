#pragma once

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include <Common/MS5611.h>

class BarometerHandler
{
  using BarMsg = sensor_msgs::FluidPressure;

public:
  BarometerHandler(ros::NodeHandle& nh);

private:
  MS5611 barometer_;
  BarMsg bar_msg_;

  ros::Publisher bar_pub_;

  ros::Timer timer_;

  void timerCb(const ros::TimerEvent&);
};
