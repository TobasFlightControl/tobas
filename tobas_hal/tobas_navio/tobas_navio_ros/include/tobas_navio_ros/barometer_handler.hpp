#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <std_srvs/Trigger.h>

#include <tobas_navio_core/ms5611.hpp>
#include <tobas_tools/node.hpp>

namespace tobas_navio_ros
{
class BarometerHandler : public tobas::BaseNode
{
  // Constants
  static constexpr size_t kSamplingRate = 50;  // [Hz]

  // Defaults
  static constexpr double kDefaultPressureNoiseDensity = 1.;  // [Pa/sqrt(Hz)]

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

  ros::Publisher bar_pub_;
  ros::ServiceServer reload_config_srv_;
  ros::Timer main_timer_;

  bool reloadConfig();

  bool reloadConfigCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros
