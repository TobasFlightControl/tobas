#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <std_srvs/Trigger.h>

#include <tobas_navio_core/adc.hpp>
#include <tobas_tools/node.hpp>

namespace tobas_navio_ros
{
class BatteryHandler : public tobas::BaseNode
{
  // Constants
  static constexpr size_t kSamplingRate = 100;     // [Hz]
  static constexpr double kLpfTimeConst = 10.;     // [s]
  static constexpr double kVoltageThreshold = 3.;  // [V]

  // Defaults
  static constexpr double kDefaultAdcCoef = 11.3;

  using self = BatteryHandler;
  using super = tobas::BaseNode;

public:
  explicit BatteryHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  navio::ADC adc_;

  // Config
  double adc_coef_;

  ros::Publisher battery_pub_;
  ros::ServiceServer reload_config_srv_;
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool reloadConfig();

  bool reloadConfigCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros
