#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <dh_ros_tools/node.hpp>

#include <static_state_determination/StaticStateDeterminationAction.h>

namespace static_state_determination
{
class StaticStateDeterminationServer : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using BarMsg = sensor_msgs::FluidPressure;
  using GpsMsg = sensor_msgs::NavSatFix;
  using VelMsg = tobas_msgs::LinearVelocityWithCovariance;

  using ActionType = static_state_determination::StaticStateDeterminationAction;
  using GoalType = static_state_determination::StaticStateDeterminationGoalConstPtr;
  using ResultType = static_state_determination::StaticStateDeterminationResult;
  using FeedbackType = static_state_determination::StaticStateDeterminationFeedback;

public:
  explicit StaticStateDeterminationServer();

private:
  GoalType goal_;
  ResultType result_;
  FeedbackType feedback_;
  bool is_action_running_;

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
  bool isValidGoal();
  bool isValidResult();

  void imuCb(const ImuMsg& imu);
  void magCb(const MagMsg& mag);
  void barCb(const BarMsg& bar);
  void gpsCb(const GpsMsg& gps);
  void velCb(const VelMsg& vel);

  void executeCb(const GoalType& goal);
};
}  // namespace static_state_determination
