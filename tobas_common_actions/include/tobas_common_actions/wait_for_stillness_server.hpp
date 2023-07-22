#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_tools/node.hpp>

#include <tobas_common_actions/WaitForStillnessAction.h>

namespace tobas_common_actions
{
class WaitForStillnessServer : public tobas::BaseNode
{
  static constexpr char kActionName[] = "wait_for_stillness";

  using super = tobas::BaseNode;

  using ActionType = tobas_common_actions::WaitForStillnessAction;
  using GoalType = tobas_common_actions::WaitForStillnessGoalConstPtr;
  using ResultType = tobas_common_actions::WaitForStillnessResult;
  using FeedbackType = tobas_common_actions::WaitForStillnessFeedback;

public:
  explicit WaitForStillnessServer();

private:
  bool is_action_running_;
  bool is_history_filled_;           // 時間窓分だけ履歴が溜まっている場合にtrue
  ros::Time t_last_valid_velocity_;  // 最後に速度が閾値内に入った時刻
  std::deque<tobas_msgs::BaseState> bs_history_;
  GoalType goal_;
  ResultType result_;

  ros::Subscriber bs_sub_;

  actionlib::SimpleActionServer<ActionType> as_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void reset();
  bool isValidGoal(const GoalType& goal);
  bool isConditionsMet();
  void fillResult();

  void eventCb(const tobas_msgs::Event& event) override;
  void baseStateCb(const tobas_msgs::BaseState& bs);

  void executeCb(const GoalType& goal);
};
}  // namespace tobas_common_actions
