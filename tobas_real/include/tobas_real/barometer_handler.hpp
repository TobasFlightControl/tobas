#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <sensor_msgs/FluidPressure.h>
#include <Common/MS5611.h>

#include <tobas_tools/node.hpp>

namespace tobas_real
{
class BarometerHandler : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 50.;  // [Hz]

  using super = tobas::BaseNode;

public:
  explicit BarometerHandler(ros::NodeHandle nh, ros::NodeHandle pnh);

private:
  MS5611 barometer_;
  sensor_msgs::FluidPressure bar_msg_;

  // Config
  double pressure_noise_density_;  // [Pa/sqrt(Hz)]

  // Publisher
  ros::Publisher bar_pub_;

  // Timer
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void readConfig();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_real
