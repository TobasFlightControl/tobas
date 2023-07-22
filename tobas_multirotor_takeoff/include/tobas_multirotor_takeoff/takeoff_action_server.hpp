#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_msgs/TakeoffAction.h>
#include <tobas_common_actions/WaitForStillnessAction.h>

namespace tobas_multirotor_takeoff
{
/**
 * @brief マルチコプターの離陸指令を発行するアクションサーバ．
 * X,Y,Yawをアクション開始時の値に保ったままZのみを増やしていく．
 * cf. https://docs.px4.io/main/en/flight_modes/takeoff.html
 */
class MultirotorTakeoffServer : public tobas::BaseNode
{
  static constexpr char kActionName[] = "multirotor_takeoff";
  static constexpr double kUpdateRate = 100.;                 // [Hz]
  static constexpr double kWaitForExternalActionServer = 3.;  // [s]

  // TODO: ActionGoalで指定できるように
  static constexpr double kInitElevation = -3.;    // [m]
  static constexpr double kTargetElevation = 2.5;  // [m]
  static constexpr double kElevationSpeed = 1.5;   // [m]

  using super = tobas::BaseNode;

  using ActionType = tobas_msgs::TakeoffAction;
  using GoalType = tobas_msgs::TakeoffGoalConstPtr;  // Goalはポインタの必要あり
  using ResultType = tobas_msgs::TakeoffResult;
  using FeedbackType = tobas_msgs::TakeoffFeedback;

public:
  explicit MultirotorTakeoffServer();

private:
  tobas_msgs::PositionYaw pos_yaw_;
  ResultType result_;
  tobas_common_actions::WaitForStillnessGoal wait_for_stillness_goal_;

  ros::Publisher pos_yaw_pub_;

  actionlib::SimpleActionServer<ActionType> as_;
  actionlib::SimpleActionClient<tobas_common_actions::WaitForStillnessAction> wait_for_stillness_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::Event& event) override;
  void executeCb(const GoalType& goal);
};
}  // namespace tobas_multirotor_takeoff
