#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/VelocityYaw.h>
#include <tobas_msgs/LandAction.h>

namespace tobas_state_checker
{
class MultirotorStateChecker : public tobas::BaseNode
{
  static constexpr char kLandActionName[] = "multirotor_landing";

  using super = tobas::BaseNode;

public:
  explicit MultirotorStateChecker();

  void run();

private:
  // rosparams
  double warn_voltage_;   // 警告を出すバッテリー電圧
  double fatal_voltage_;  // 飛行を停止するバッテリー電圧

  bool battery_received_;
  bool bs_received_;
  bool cmd_received_;
  tobas_msgs::Battery battery_;
  tobas_msgs::BaseState bs_;
  ros::Time t_last_bs_;
  ros::Time t_last_cmd_;
  tobas_msgs::Event event_;

  ros::Publisher event_pub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber bs_sub_;
  ros::Subscriber cmd_sub_;

  actionlib::SimpleActionClient<tobas_msgs::LandAction> ac_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void requestLanding();
  void requestShutdown();

  void eventCb(const tobas_msgs::Event& event) override;
  void batteryCb(const tobas_msgs::Battery& battery);
  void baseStateCb(const tobas_msgs::BaseState& bs);
  void commandCb(const tobas_msgs::VelocityYaw& cmd);
};
}  // namespace tobas_state_checker
