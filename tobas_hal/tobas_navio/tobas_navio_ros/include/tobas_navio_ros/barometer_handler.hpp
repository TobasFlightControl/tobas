#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

#include <tobas_navio_core/ms5611.hpp>
#include <tobas_tools/node.hpp>

namespace tobas_navio_ros
{
class BarometerHandler : public tobas::BaseNode
{
  // Default Parameters
  static constexpr size_t kDefaultUpdateRate = 50;  // [Hz]

  using self = BarometerHandler;
  using super = tobas::BaseNode;

public:
  explicit BarometerHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  navio::MS5611 barometer_;

  // Config
  double pressure_noise_density_;  // [Pa/sqrt(Hz)]

  // rosparam
  size_t update_rate_;

  // Publisher
  ros::Publisher bar_pub_;

  // Timer
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void readConfig();

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros
