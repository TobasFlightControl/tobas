#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

#include <tobas_navio_core/pwm.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PwmArray.h>
#include <tobas_msgs/EnablePwm.h>

#include "./common.hpp"

namespace tobas_navio_ros
{
class PwmHandler : public tobas::BaseNode
{
  using self = PwmHandler;
  using super = tobas::BaseNode;

public:
  explicit PwmHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name = ros::this_node::getName());

  ~PwmHandler();

private:
  navio::PWM pwm_;
  std::array<bool, kServoRailSize> is_enabled_;

  // Subscribers
  ros::Subscriber pwms_sub_;

  // Service Servers
  ros::ServiceServer enable_pwm_srv_;

  void pwmsCb(const tobas_msgs::PwmArrayConstPtr& pwms);
  bool enablePwmCb(tobas_msgs::EnablePwmRequest& req, tobas_msgs::EnablePwmResponse& res);
};
}  // namespace tobas_navio_ros
