#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

#include <Navio2/PWM.h>
#include <tobas_tools/node.hpp>
#include <tobas_msgs/PwmArray.h>
#include <tobas_msgs/InitializePwm.h>
#include <tobas_msgs/EnablePwm.h>
#include <tobas_msgs/SetPwmFrequency.h>

#include "./common.hpp"

namespace tobas_real
{
struct PwmState
{
  bool exported = false;
  bool enabled = false;
  double period = 0;  // [us]
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
  std::array<PwmState, kServoRailSize> pwm_states_;  // Channel -> PWM State

  // Subscribers
  ros::Subscriber pwms_sub_;

  // Service Servers
  ros::ServiceServer initialize_srv_;
  ros::ServiceServer enable_srv_;
  ros::ServiceServer set_freq_srv_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;
  void registerServiceServers();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void pwmsCb(const tobas_msgs::PwmArrayConstPtr& pwms);

  bool initializeCb(tobas_msgs::InitializePwmRequest& req, tobas_msgs::InitializePwmResponse& res);
  bool enableCb(tobas_msgs::EnablePwmRequest& req, tobas_msgs::EnablePwmResponse& res);
  bool setFreqCb(tobas_msgs::SetPwmFrequencyRequest& req, tobas_msgs::SetPwmFrequencyResponse& res);
};
}  // namespace tobas_real
