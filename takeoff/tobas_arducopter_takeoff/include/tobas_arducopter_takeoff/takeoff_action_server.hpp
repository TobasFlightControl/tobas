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

  static constexpr double kWaitForService = 1.;            // [s]
  static constexpr double kWaitForArming = 1.;             // [s]
  static constexpr double kRetryInterval = 3.;             // [s]
  static constexpr double kTakeoffCheckAltThreshold = 1.5;  // [m]

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
  ResultType result_;
  ros::Time action_called_time_;

  ros::Subscriber pt_sub_;

  ros::ServiceClient set_mode_sc_;
  ros::ServiceClient arming_sc_;
  ros::ServiceClient takeoff_sc_;

  actionlib::SimpleActionServer<ActionType> as_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isGoalValid(const GoalType& goal);
  bool waitForServiceExistence();
  bool waitForPoseTwistReceived(const double& timeout);
  bool setMode(const double& timeout);
  bool arming(const double& timeout);
  bool takeoff(const double& timeout, const double& target_altitude);
  void setSucceeded();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt);

  void executeCb(const GoalType& goal);
};
}  // namespace tobas_arducopter_takeoff
