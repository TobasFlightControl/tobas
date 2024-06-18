#pragma once

#include <std_srvs/Trigger.h>

#include <tobas_std_tools/range.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_navio_core/rc_input.hpp>

#include "./base_sensor_node.hpp"

namespace tobas_navio_ros
{
class RCInputHandler : public BaseSensorNode
{
  static constexpr size_t kSamplingRate = 100;  // [Hz]

  using self = RCInputHandler;
  using super = BaseSensorNode;

public:
  explicit RCInputHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  navio::RCInput rcin_;

  // Config
  tobas_std::Range<double> roll_range_;
  tobas_std::Range<double> pitch_range_;
  tobas_std::Range<double> yaw_range_;
  tobas_std::Range<double> throttle_range_;
  std::array<double, tobas::kNumFlightModes> modes_;
  double mode_auto_, mode_position_, mode__;
  double estop_on_, estop_off_;
  double gpsw_on_, gpsw_off_;

  ros::Publisher rcin_pub_;
  ros::ServiceServer reload_config_srv_;

  bool reloadConfig();

  bool reloadConfigCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros
