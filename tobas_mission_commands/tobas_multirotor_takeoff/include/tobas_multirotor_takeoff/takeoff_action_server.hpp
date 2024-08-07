#pragma once

#include <rclcpp/rclcpp.hpp>
#include <actionlib/server/simple_action_server.h>

#include <tobas_node/node.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/TakeoffAction.h>

namespace tobas_multirotor_takeoff
{
/**
 * @brief マルチコプターの離陸指令を発行するアクションサーバ．
 * X,Y,Yawをアクション開始時の値に保ったままZのみを増やしていく．
 * cf. https://docs.px4.io/main/en/flight_modes/takeoff.html
 */
class TakeoffActionServer : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 100.;  // [Hz]

  using self = TakeoffActionServer;
  using super = tobas::BaseNode;

  using ActionType = tobas_msgs::TakeoffAction;
  using GoalType = tobas_msgs::TakeoffGoal;
  using ResultType = tobas_msgs::TakeoffResult;
  using FeedbackType = tobas_msgs::TakeoffFeedback;

public:
  explicit TakeoffActionServer(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  ResultType result_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;

  PublisherPtr<> cmd_pub_;
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  rclcpp::ServiceClient set_arm_sc_;
  actionlib::SimpleActionServer<ActionType> as_;

  bool isGoalValid(const GoalType& goal);
  bool armRotors();

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void executeCb(const GoalType::ConstSharedPtr& goal);
};
}  // namespace tobas_multirotor_takeoff
