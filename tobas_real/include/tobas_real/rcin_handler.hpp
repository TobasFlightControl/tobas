#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <Navio2/RCInput_Navio2.h>

#include <tobas_std_tools/range.hpp>

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
  RCInput_Navio2 rcin_;

  // Config
  tobas_std::Range<double> roll_range_;
  tobas_std::Range<double> pitch_range_;
  tobas_std::Range<double> yaw_range_;
  tobas_std::Range<double> thrust_range_;
  size_t num_modes_;
  std::vector<double> modes_;
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
