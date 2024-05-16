#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/PositionYaw.h>

#include <tobas_trajectory_commander/FollowPositionYawTrajectoryAction.h>

namespace tobas_trajectory_commander
{
class FollowPositionYawTrajectoryServer : tobas::BaseNode
{
  static constexpr char kActionName[] = "follow_trajectory_position_yaw";
  static constexpr size_t kMaxNrOfTrajPoint = 1000;  // TODO: メモリ的な限界値を探す

  using self = FollowPositionYawTrajectoryServer;
  using super = tobas::BaseNode;

  using CommandType = tobas_msgs::PositionYaw;

  using ActionType = tobas_trajectory_commander::FollowPositionYawTrajectoryAction;
  using GoalType = tobas_trajectory_commander::FollowPositionYawTrajectoryGoal;
  using ResultType = tobas_trajectory_commander::FollowPositionYawTrajectoryResult;
  using FeedbackType = tobas_trajectory_commander::FollowPositionYawTrajectoryFeedback;

public:
  explicit FollowPositionYawTrajectoryServer(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  ResultType result_;

  ros::Publisher cmd_pub_;
  ros::Subscriber event_sub_;
  actionlib::SimpleActionServer<ActionType> as_;

  bool isGoalValid(const GoalType& goal);
  void executeCb(const GoalType::ConstPtr& goal);
};
}  // namespace tobas_trajectory_commander
