#pragma once

#include <deque>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <dh_ros_tools/node.hpp>

#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_trajectory_commander/LandAction.h>

namespace tobas_trajectory_commander
{
class MultirotorLandServer : public dh_ros::BaseNode
{
  static constexpr char kActionName[] = "multirotor_land";
  static constexpr double kUpdateRate = 100.;    // [Hz]
  static constexpr double kVerticalSpeed = 0.1;  // [m/s]
  static constexpr double kTimeWindow = 3.;      // [s] 高度の変化を見る時間窓の長さ
  static constexpr double kStableAltitudeRange = 0.05;  // [m]

  using super = dh_ros::BaseNode;

  using ActionType = tobas_trajectory_commander::LandAction;
  using GoalType = tobas_trajectory_commander::LandGoalConstPtr;
  using ResultType = tobas_trajectory_commander::LandResult;
  using FeedbackType = tobas_trajectory_commander::LandFeedback;

public:
  explicit MultirotorLandServer();

private:
  bool is_action_running_;
  bool bs_received_;
  bool is_history_filled_;  // 時間窓分だけ履歴が溜まっている場合にtrue
  std::deque<std::pair<ros::Time, double>> alt_history_;
  tobas_msgs::BaseState bs_;
  tobas_msgs::PositionYaw cmd_;
  ResultType result_;

  ros::Publisher cmd_pub_;
  ros::Subscriber bs_sub_;

  actionlib::SimpleActionServer<ActionType> as_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void reset();

  void baseStateCb(const tobas_msgs::BaseState& bs);
  void executeCb(const GoalType&);
};
}  // namespace tobas_trajectory_commander
