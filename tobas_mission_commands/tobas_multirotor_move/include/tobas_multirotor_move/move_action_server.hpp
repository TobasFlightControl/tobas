#pragma once

#include <rclcpp/rclcpp.hpp>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/msg/bool.hpp>

#include <tobas_node/node.hpp>
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

  std_msgs::msg::Bool::ConstSharedPtr arming_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;

  PublisherPtr<> cmd_pub_;
  SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;

  actionlib::SimpleActionServer<ActionType> as_;

  bool isGoalValid(const GoalType& goal);
  bool computeGoalPosition(const GoalType& goal, kdl::Vector& goal_pos);

  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);

  void executeCb(const GoalType::ConstSharedPtr& goal);
};
}  // namespace tobas_multirotor_move
