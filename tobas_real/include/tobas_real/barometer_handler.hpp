#pragma once

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include <Common/MS5611.h>

#include <dh_ros_tools/node.hpp>

class BarometerHandler : public dh_ros::BaseNode
{
  using BarMsg = sensor_msgs::FluidPressure;

public:
  BarometerHandler();

private:
  MS5611 barometer_;
  BarMsg bar_msg_;

  // roaparams
  std::string drone_name_;

  // PubSub
  ros::Publisher bar_pub_;

  ros::Timer main_loop_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;
  void createTimers() override;

  void checkTopicsTimerCb(const ros::TimerEvent& event) override;
  void mainLoopTimerCb(const ros::TimerEvent&);
};
