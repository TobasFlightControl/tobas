#pragma once

#include <rclcpp/rclcpp.hpp>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Bool.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/MoveAction.h>

namespace tobas_multirotor_move
{
class MoveActionServer : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 100.;  // [Hz]

  using self = MoveActionServer;
  using super = tobas::BaseNode;

  using ActionType = tobas_msgs::MoveAction;
  using GoalType = tobas_msgs::MoveGoal;
  using ResultType = tobas_msgs::MoveResult;
  using FeedbackType = tobas_msgs::MoveFeedback;

public:
  explicit MoveActionServer(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  ResultType result_;

  std_msgs::BoolConstPtr arming_;
  tobas_msgs::OdometryConstPtr odom_;

  rclcpp::Publisher cmd_pub_;
  rclcpp::Subscriber arming_sub_;
  rclcpp::Subscriber odom_sub_;

  actionlib::SimpleActionServer<ActionType> as_;

  bool isGoalValid(const GoalType& goal);
  bool computeGoalPosition(const GoalType& goal, kdl::Vector& goal_pos);

  void armingCb(const std_msgs::BoolConstPtr& arming);
  void odomCb(const tobas_msgs::OdometryConstPtr& odom);

  void executeCb(const GoalType::ConstPtr& goal);
};
}  // namespace tobas_multirotor_move
