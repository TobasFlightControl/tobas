#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/VelocityYaw.h>
#include <tobas_trajectory_commander/LandAction.h>

namespace tobas_state_checker
{
class MultirotorStateChecker : public tobas::BaseNode
{
  static constexpr char kLandActionName[] = "multirotor_land";
  static constexpr double kSleepTime = 0.1;                // [s]
  static constexpr double kWaitForActionServer = 3.;       // [s]
  static constexpr double kCommandTimeoutThreshold = 0.5;  // [s]

  using super = tobas::BaseNode;

public:
  explicit MultirotorStateChecker();

  void run();

private:
  bool bs_received_;
  bool cmd_received_;
  ros::Time t_last_cmd_;

  ros::Publisher event_pub_;
  ros::Subscriber bs_sub_;
  ros::Subscriber cmd_sub_;

  actionlib::SimpleActionClient<tobas_trajectory_commander::LandAction> ac_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::Event& event) override;
  void baseStateCb(const tobas_msgs::BaseState&);
  void commandCb(const tobas_msgs::VelocityYaw& cmd);
};
}  // namespace tobas_state_checker
