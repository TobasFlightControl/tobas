#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <Navio2/RCInput_Navio2.h>

#include <dh_std_tools/range.hpp>

#include <tobas_tools/node.hpp>

namespace tobas_real
{
class RCInputHandler : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 100.;  // [Hz]

  using super = tobas::BaseNode;

public:
  explicit RCInputHandler(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  RCInput_Navio2 rcin_;

  // RC input period ranges
  dh_std::Range<double> roll_range_;
  dh_std::Range<double> pitch_range_;
  dh_std::Range<double> yaw_range_;
  dh_std::Range<double> thrust_range_;
  dh_std::Range<double> toggle_range_;

  // Publisher
  ros::Publisher rcin_pub_;

  // Timer
  ros::Timer main_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void getRcPeriodRanges();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_real
