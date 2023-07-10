#pragma once

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <Common/MS5611.h>

#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>

namespace tobas_real
{
class BarometerHandler : public tobas::BaseNode
{
  using super = tobas::BaseNode;

  using BarMsg = sensor_msgs::FluidPressure;

public:
  explicit BarometerHandler();

private:
  MS5611 barometer_;
  BarMsg bar_msg_;

  // PubSub
  ros::Publisher bar_pub_;

  // Timer
  dh_ros::Timer main_loop_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::Event& event) override;
  void mainLoopTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_real
