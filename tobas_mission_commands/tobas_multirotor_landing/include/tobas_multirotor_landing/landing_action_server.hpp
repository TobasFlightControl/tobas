#pragma once

#include <rclcpp/rclcpp.hpp>
#include <actionlib/server/simple_action_server.h>

#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_node/node.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/LandAction.h>

namespace tobas_multirotor_landing
{
class LandActionServer : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 100.;           // [Hz]
  static constexpr double kVerticalSpeed = 0.3;         // [m/s]
  static constexpr double kTimeWindow = 5.;             // [s] 高度の変化を見る時間窓の長さ
  static constexpr double kStableAltitudeRange = 0.03;  // [m]

  using self = LandActionServer;
  using super = tobas::BaseNode;

  using ActionType = tobas_msgs::LandAction;
  using GoalType = tobas_msgs::LandGoal;
  using ResultType = tobas_msgs::LandResult;
  using FeedbackType = tobas_msgs::LandFeedback;

public:
  explicit LandActionServer(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  bool is_action_running_ = false;
  tobas_std::TimestampedBuffer<double> alt_buf_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  ResultType result_;

  PublisherPtr<> cmd_pub_;
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;

  rclcpp::ServiceClient set_arm_sc_;
  actionlib::SimpleActionServer<ActionType> as_;

  bool disarmRotors();

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void executeCb(const GoalType::ConstSharedPtr& goal);
};
}  // namespace tobas_multirotor_landing
