#pragma once

#include <ros/ros.h>
#include <Navio2/RCInput_Navio2.h>

#include <dh_std_tools/range.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/RCInput.h>

namespace tobas_real
{
class RCInputHandler : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 100.;  // [Hz]

  using super = tobas::BaseNode;

public:
  explicit RCInputHandler(ros::NodeHandle nh, ros::NodeHandle pnh);

  void run();

private:
  RCInput_Navio2 rcin_;
  tobas_msgs::RCInput rcin_msg_;

  // RC input period ranges
  dh_std::Range<double> roll_range_;
  dh_std::Range<double> pitch_range_;
  dh_std::Range<double> yaw_range_;
  dh_std::Range<double> thrust_range_;
  dh_std::Range<double> toggle_range_;

  // Publisher
  ros::Publisher rcin_pub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void getRcPeriodRanges();

  void eventCb(const tobas_msgs::Event& event) override;
};
}  // namespace tobas_real
