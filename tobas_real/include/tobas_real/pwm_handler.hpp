#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

#include <Navio2/PWM.h>
#include <tobas_tools/node.hpp>
#include <tobas_msgs/PwmArray.h>
#include <tobas_msgs/SetupPwm.h>

#include "./common.hpp"

namespace tobas_real
{
class PwmHandler : public tobas::BaseNode
{
  static constexpr bool kDefaultUnexportWhenShutdown = false;

  using self = PwmHandler;
  using super = tobas::BaseNode;

public:
  explicit PwmHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

  ~PwmHandler();

private:
  PWM pwm_;
  std::array<bool, kServoRailSize> pwm_ok_;

  // rosparams
  bool unexport_when_shutdown_;

  // Subscribers
  ros::Subscriber pwms_sub_;

  // Service Servers
  ros::ServiceServer setup_pwm_srv_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;
  void registerServiceServers();

  void pwmsCb(const tobas_msgs::PwmArrayConstPtr& pwms);

  bool setupPwmCb(tobas_msgs::SetupPwmRequest& req, tobas_msgs::SetupPwmResponse& res);
};
}  // namespace tobas_real
