#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/PoseTwist.h>

#include <tobas_msgs/TakeoffAction.h>

namespace tobas_arducopter_takeoff
{
class TakeoffActionServer : public tobas::BaseNode
{
  const std::string kSetModeSrvName = "mavros/set_mode";
  const std::string kArmingSrvName = "mavros/cmd/arming";
  const std::string kTakeoffSrvName = "mavros/cmd/takeoff";

  static constexpr double kWaitForService = 1.;         // [s]
  static constexpr double kWaitForArming = 1.;          // [s]
  static constexpr double kRetryInterval = 3.;          // [s]
  static constexpr double kTakeoffCheckThreshold = 2.;  // [m]

  // TODO: ActionGoalで指定できるように
  static constexpr double kTargetElevation = 5.;  // Check: kTakeoffCheckThresholdよりは大きい
  static constexpr double kTimeout = 1e+9;

  using self = TakeoffActionServer;
  using super = tobas::BaseNode;

  using ActionType = tobas_msgs::TakeoffAction;
  using GoalType = tobas_msgs::TakeoffGoalConstPtr;
  using ResultType = tobas_msgs::TakeoffResult;
  using FeedbackType = tobas_msgs::TakeoffFeedback;

public:
  explicit TakeoffActionServer(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  tobas_msgs::PoseTwistConstPtr pt_;

  ros::Subscriber pt_sub_;

  ros::ServiceClient set_mode_ac_;
  ros::ServiceClient arming_ac_;
  ros::ServiceClient takeoff_ac_;

  actionlib::SimpleActionServer<ActionType> as_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt);

  void executeCb(const GoalType& goal);
};
}  // namespace tobas_arducopter_takeoff
