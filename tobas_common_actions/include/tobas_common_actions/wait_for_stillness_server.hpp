#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/WaitForStillnessAction.h>

namespace tobas_common_actions
{
class WaitForStillnessServer : public tobas::BaseNode
{
  static constexpr char kActionName[] = "wait_for_stillness";

  using super = tobas::BaseNode;

  using ActionType = tobas_msgs::WaitForStillnessAction;
  using GoalType = tobas_msgs::WaitForStillnessGoalConstPtr;
  using ResultType = tobas_msgs::WaitForStillnessResult;
  using FeedbackType = tobas_msgs::WaitForStillnessFeedback;

public:
  explicit WaitForStillnessServer(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  bool is_action_running_;
  bool is_history_filled_;           // 時間窓分だけ履歴が溜まっている場合にtrue
  ros::Time t_last_valid_attitude_;  // 最後に姿勢角が閾値内に入った時刻
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

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void baseStateCb(const tobas_msgs::BaseStateConstPtr& bs);

  void executeCb(const GoalType& goal);
};
}  // namespace tobas_common_actions
