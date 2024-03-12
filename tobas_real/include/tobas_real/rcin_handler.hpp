#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <Navio2/RCInput.h>

#include <tobas_std_tools/range.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_tools/node.hpp>

namespace tobas_real
{
class RCInputHandler : public tobas::BaseNode
{
  static constexpr size_t kUpdateRate = 100;    // [Hz]
  static constexpr double kSignalMargin = 0.1;  // [-]

  using super = tobas::BaseNode;

public:
  explicit RCInputHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  RCInput rcin_;

  // Config
  tobas_std::Range<int> roll_range_;
  tobas_std::Range<int> pitch_range_;
  tobas_std::Range<int> yaw_range_;
  tobas_std::Range<int> thrust_range_;
  std::array<int, tobas::kNumFlightModes> modes_;
  double mode_auto_, mode_position_, mode__;
  double estop_on_, estop_off_;
  double gpsw_on_, gpsw_off_;

  // Publisher
  ros::Publisher rcin_pub_;

  // Timer
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void readConfig();

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_real
