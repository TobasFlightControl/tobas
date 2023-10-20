#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>

#include <tobas_tools/node.hpp>

#include <tobas_msgs/TakeoffAction.h>

namespace tobas_arducopter_takeoff
{
class TakeoffActionServer : public tobas::BaseNode
{
  const std::string kSetModeSrvName = "mavros/set_mode";
  const std::string kArmingSrvName = "mavros/cmd/arming";
  const std::string kTakeoffSrvName = "mavros/cmd/takeoff";

  static constexpr double kWaitForService = 3.;  // [s]

  // TODO: ActionGoalで指定できるように
  static constexpr double kTargetElevation = 2.;  // [m]

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
  mavros_msgs::SetModeRequest set_mode_req_;
  mavros_msgs::SetModeResponse set_mode_res_;
  mavros_msgs::CommandBoolRequest arming_req_;
  mavros_msgs::CommandBoolResponse arming_res_;
  mavros_msgs::CommandTOLRequest takeoff_req_;
  mavros_msgs::CommandTOLResponse takeoff_res_;

  ros::ServiceClient set_mode_ac_;
  ros::ServiceClient arming_ac_;
  ros::ServiceClient takeoff_ac_;

  actionlib::SimpleActionServer<ActionType> as_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::EventConstPtr& event) override;

  void executeCb(const GoalType& goal);
};
}  // namespace tobas_arducopter_takeoff
