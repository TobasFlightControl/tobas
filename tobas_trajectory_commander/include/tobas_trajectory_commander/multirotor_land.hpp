#pragma once

#include <deque>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_trajectory_commander/LandAction.h>

namespace tobas_trajectory_commander
{
class MultirotorLandServer : public tobas::BaseNode
{
  static constexpr char kActionName[] = "multirotor_land";
  static constexpr double kUpdateRate = 100.;  // [Hz]
  static constexpr double kVerticalSpeed = 1.;  // [m/s] 多くのドローンでは1~2mらしい (GPT4)
  static constexpr double kTimeWindow = 3.;  // [s] 高度の変化を見る時間窓の長さ
  static constexpr double kStableAltitudeRange = 0.03;  // [m]

  using super = tobas::BaseNode;

  using ActionType = tobas_trajectory_commander::LandAction;
  using GoalType = tobas_trajectory_commander::LandGoalConstPtr;  // Goalはポインタの必要あり
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

  void eventCb(const tobas_msgs::Event& event) override;
  void baseStateCb(const tobas_msgs::BaseState& bs);
  void executeCb(const GoalType& goal);
};
}  // namespace tobas_trajectory_commander
