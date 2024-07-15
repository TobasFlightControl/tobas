#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

#include <tobas_navio_core/pwm.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/ThrottleArray.h>
#include <tobas_msgs/EnableRCOutput.h>

#include "./common.hpp"

namespace tobas_navio_ros
{
class PwmHandler : public tobas::BaseNode
{
  static constexpr size_t kPwmFrequency = 400;  // [Hz] PX4のデフォルト値

  using self = PwmHandler;
  using super = tobas::BaseNode;

public:
  explicit PwmHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name = ros::this_node::getName());

  ~PwmHandler();

private:
  navio::PWM pwm_;
  std::array<bool, navio::PWM::kChannelCount> is_enabled_;

  ros::Subscriber throttles_sub_;
  ros::ServiceServer enable_rcout_srv_;

  void throttlesCb(const tobas_msgs::ThrottleArrayConstPtr& throttles);
  bool enableRCOutputCb(tobas_msgs::EnableRCOutputRequest& req, tobas_msgs::EnableRCOutputResponse& res);
};
}  // namespace tobas_navio_ros
