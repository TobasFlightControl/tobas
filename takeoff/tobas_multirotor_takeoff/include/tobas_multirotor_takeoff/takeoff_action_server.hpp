#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Odometry.h>

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
  static constexpr double kUpdateRate = 100.;                 // [Hz]
  static constexpr double kWaitForExternalActionServer = 3.;  // [s]

  using self = TakeoffActionServer;
  using super = tobas::BaseNode;

  using ActionType = tobas_msgs::TakeoffAction;
  using GoalType = tobas_msgs::TakeoffGoalConstPtr;
  using ResultType = tobas_msgs::TakeoffResult;
  using FeedbackType = tobas_msgs::TakeoffFeedback;

public:
  explicit TakeoffActionServer(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas_msgs::OdometryConstPtr odom_;
  ResultType result_;

  ros::Publisher cmd_pub_;
  ros::Subscriber odom_sub_;

  actionlib::SimpleActionServer<ActionType> as_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isGoalValid(const GoalType& goal);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void odomCb(const tobas_msgs::OdometryConstPtr& odom);

  void executeCb(const GoalType& goal);
};
}  // namespace tobas_multirotor_takeoff
