#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/LandAction.h>

namespace tobas_multirotor_landing
{
class LandActionServer : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 100.;    // [Hz]
  static constexpr double kVerticalSpeed = 0.3;  // [m/s]
  static constexpr double kTimeWindow = 5.;      // [s] 高度の変化を見る時間窓の長さ
  static constexpr double kStableAltitudeRange = 0.03;  // [m]

  using self = LandActionServer;
  using super = tobas::BaseNode;

  using ActionType = tobas_msgs::LandAction;
  using GoalType = tobas_msgs::LandGoal;
  using ResultType = tobas_msgs::LandResult;
  using FeedbackType = tobas_msgs::LandFeedback;

public:
  explicit LandActionServer(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  bool is_action_running_;
  bool is_history_filled_;  // 時間窓分だけ履歴が溜まっている場合にtrue
  std::deque<std::pair<ros::Time, double>> alt_history_;
  tobas_msgs::OdometryConstPtr odom_;
  ResultType result_;

  ros::Publisher cmd_pub_;
  ros::Subscriber odom_sub_;

  ros::ServiceClient set_arm_sc_;
  actionlib::SimpleActionServer<ActionType> as_;

  void reset();
  bool disarmRotors();

  void odomCb(const tobas_msgs::OdometryConstPtr& odom);

  void executeCb(const GoalType::ConstPtr& goal);
};
}  // namespace tobas_multirotor_landing
