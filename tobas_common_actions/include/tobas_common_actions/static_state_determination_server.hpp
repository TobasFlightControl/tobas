#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_tools/node.hpp>

#include <tobas_common_actions/StaticStateDeterminationAction.h>

namespace tobas_common_actions
{
class StaticStateDeterminationServer : public tobas::BaseNode
{
  static constexpr char kActionName[] = "static_state_determination";

  using super = tobas::BaseNode;

  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using BarMsg = sensor_msgs::FluidPressure;
  using GpsMsg = sensor_msgs::NavSatFix;
  using VelMsg = tobas_msgs::LinearVelocityWithCovariance;

  using ActionType = tobas_common_actions::StaticStateDeterminationAction;
  using GoalType = tobas_common_actions::StaticStateDeterminationGoalConstPtr;
  using ResultType = tobas_common_actions::StaticStateDeterminationResult;
  using FeedbackType = tobas_common_actions::StaticStateDeterminationFeedback;

public:
  explicit StaticStateDeterminationServer();

private:
  GoalType goal_;
  ResultType result_;
  FeedbackType feedback_;
  bool is_action_running_;
  uint32_t imu_count_;
  uint32_t mag_count_;
  uint32_t bar_count_;
  uint32_t gps_count_;
  uint32_t vel_count_;
  ImuMsg imu_sum_;
  MagMsg mag_sum_;
  BarMsg bar_sum_;
  GpsMsg gps_sum_;
  VelMsg vel_sum_;

  ros::Subscriber imu_sub_;
  ros::Subscriber mag_sub_;
  ros::Subscriber bar_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber vel_sub_;

  actionlib::SimpleActionServer<ActionType> as_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void reset();
  void fillResult();
  bool isValidGoal();
  bool isValidResult();

  void eventCb(const tobas_msgs::Event& event) override;
  void imuCb(const ImuMsg& imu);
  void magCb(const MagMsg& mag);
  void barCb(const BarMsg& bar);
  void gpsCb(const GpsMsg& gps);
  void velCb(const VelMsg& vel);

  void executeCb(const GoalType& goal);
};
}  // namespace tobas_common_actions
