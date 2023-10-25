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
  static constexpr uint32_t kUpdateRate = 100;  // [Hz]

  using super = tobas::BaseNode;

public:
  explicit RCInputHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  RCInput_Navio2 rcin_;

  // Config
  dh_std::Range<double> roll_range_;
  dh_std::Range<double> pitch_range_;
  dh_std::Range<double> yaw_range_;
  dh_std::Range<double> thrust_range_;
  dh_std::Range<double> estop_range_;
  std::vector<double> modes_;

  // Publisher
  ros::Publisher rcin_pub_;

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
