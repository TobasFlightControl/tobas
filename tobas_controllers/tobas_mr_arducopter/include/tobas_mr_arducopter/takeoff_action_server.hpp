#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/Bool.h>
#include <geometry_msgs/msg/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_tools/node.hpp>

#include <tobas_msgs/TakeoffAction.h>

namespace tobas_mr_arducopter
{
class TakeoffActionServer : public tobas::BaseNode
{
  const std::string kSetModeSrvName = "mavros/set_mode";
  const std::string kArmingSrvName = "mavros/cmd/arming";
  const std::string kTakeoffSrvName = "mavros/cmd/takeoff";

  static constexpr double kWaitForArming = 1.;              // [s]
  static constexpr double kRetryInterval = 3.;              // [s]
  static constexpr double kTakeoffCheckAltThreshold = 1.5;  // [m]

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
  bool is_param_server_ok_ = false;
  geometry_msgs::msg::PoseStampedConstPtr pose_;
  ResultType result_;
  rclcpp::Time action_called_time_;

  rclcpp::Subscriber local_pos_sub_;
  rclcpp::Subscriber param_server_state_sub_;

  rclcpp::ServiceClient set_mode_sc_;
  rclcpp::ServiceClient arming_sc_;
  rclcpp::ServiceClient takeoff_sc_;

  actionlib::SimpleActionServer<ActionType> as_;

  bool isGoalValid(const GoalType& goal);
  bool waitForServiceExistence();
  bool waitForParamServer(const double& timeout);
  bool setMode(const double& timeout);
  bool arming(const double& timeout);
  bool takeoff(const double& timeout, const double& target_altitude);
  void setSucceeded();

  void localPositionCb(const geometry_msgs::msg::PoseStampedConstPtr& pose);
  void paramServerStateCb(const std_msgs::BoolConstPtr& state);

  void executeCb(const GoalType::ConstPtr& goal);
};
}  // namespace tobas_mr_arducopter
