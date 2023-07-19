#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/RollPitchYawThrust.h>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_msgs/LandAction.h>

namespace tobas_multirotor_takeoff
{
class MultirotorTakeoffServer : public tobas::BaseNode
{
  static constexpr char kActionName[] = "multirotor_takeoff";
  static constexpr double kUpdateRate = 100.;              // [Hz]
  static constexpr double kTakeoffAltitudeThreshold = 1.;  // [m]
  static constexpr double kHoverAltitude = 1.5;            // [m]
  static constexpr double kThrustRate = 1.;                // [N/s]

  using super = tobas::BaseNode;

  using ActionType = tobas_msgs::LandAction;
  using GoalType = tobas_msgs::LandGoalConstPtr;  // Goalはポインタの必要あり
  using ResultType = tobas_msgs::LandResult;
  using FeedbackType = tobas_msgs::LandFeedback;

public:
  explicit MultirotorTakeoffServer();

private:
  bool bs_received_;
  tobas_msgs::BaseState bs_;
  ResultType result_;

  ros::Publisher rpy_thrust_pub_;
  ros::Publisher pos_yaw_pub_;
  ros::Subscriber bs_sub_;

  actionlib::SimpleActionServer<ActionType> as_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::Event& event) override;
  void baseStateCb(const tobas_msgs::BaseState& bs);
  void executeCb(const GoalType& goal);
};
}  // namespace tobas_multirotor_takeoff
