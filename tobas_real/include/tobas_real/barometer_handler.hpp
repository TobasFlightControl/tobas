#pragma once

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include <Common/MS5611.h>

#include <dh_ros_tools/node.hpp>

namespace tobas_real
{
class BarometerHandler : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

  using BarMsg = sensor_msgs::FluidPressure;

public:
  explicit BarometerHandler();

private:
  MS5611 barometer_;
  BarMsg bar_msg_;

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
}  // namespace tobas_real
