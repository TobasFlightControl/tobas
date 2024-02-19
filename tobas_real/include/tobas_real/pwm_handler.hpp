#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <std_srvs/SetBool.h>

#include <Navio2/PWM.h>
#include <tobas_tools/node.hpp>
#include <tobas_msgs/PwmArray.h>
#include <tobas_msgs/SetupPwm.h>
#include <tobas_msgs/EnablePwm.h>

#include "./common.hpp"

namespace tobas_real
{
struct PwmState
{
  bool is_initialized = false;
  bool is_enabled = false;
};

class PwmHandler : public tobas::BaseNode
{
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
  std::array<PwmState, kServoRailSize> pwm_states_;

  // Subscribers
  ros::Subscriber pwms_sub_;

  // Service Servers
  ros::ServiceServer setup_pwm_srv_;
  ros::ServiceServer enable_pwm_srv_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;
  void registerServiceServers();

  void pwmsCb(const tobas_msgs::PwmArrayConstPtr& pwms);

  bool setupPwmCb(tobas_msgs::SetupPwmRequest& req, tobas_msgs::SetupPwmResponse& res);
  bool enablePwmCb(tobas_msgs::EnablePwmRequest& req, tobas_msgs::EnablePwmResponse& res);
};
}  // namespace tobas_real
