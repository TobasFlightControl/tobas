#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/StaticStateDeterminationAction.h>

namespace tobas_common_actions
{
class StaticStateDeterminationServer : public tobas::BaseNode
{
  static constexpr double kMeasureTime = 5.;                // [s]
  static constexpr double kGyroNormThreshold = M_PI / 6;    // [rad/s]
  static constexpr double kAccelErrorThreshold = 1.;        // [m/s^2]
  static constexpr double kAirAltStddevThreshold = 1e+100;  // [m]  // TODO
  static constexpr double kGpsAltStddevThreshold = 5.;      // [m]
  static constexpr double kGpsPosCovStddevThreshold = 5.;   // [m]

  using self = StaticStateDeterminationServer;
  using super = tobas::BaseNode;

  using BatMsg = tobas_msgs::Battery;
  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using BarMsg = sensor_msgs::FluidPressure;
  using GpsMsg = tobas_msgs::Gps;

  using ActionType = tobas_msgs::StaticStateDeterminationAction;
  using GoalType = tobas_msgs::StaticStateDeterminationGoalConstPtr;
  using ResultType = tobas_msgs::StaticStateDeterminationResult;
  using FeedbackType = tobas_msgs::StaticStateDeterminationFeedback;

public:
  explicit StaticStateDeterminationServer(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;

  ResultType result_;
  bool is_action_running_;
  Eigen::Matrix3d gps_pos_cov_;
  tobas_std::TimestampedBufferDouble bar_alt_buf_, gps_alt_buf_;
  ros::Time t_meas_start_;
  size_t bat_count_, imu_count_, mag_count_, bar_count_, gps_count_;
  BatMsg bat_sum_;
  ImuMsg imu_sum_;
  MagMsg mag_sum_;
  BarMsg bar_sum_;
  GpsMsg gps_sum_;

  ros::Subscriber bat_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber mag_sub_;
  ros::Subscriber bar_sub_;
  ros::Subscriber gps_sub_;

  actionlib::SimpleActionServer<ActionType> as_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isConditionsMet();
  void reset();
  void fillResult();

  void batCb(const BatMsg::ConstPtr& bat);
  void imuCb(const ImuMsg::ConstPtr& imu);
  void magCb(const MagMsg::ConstPtr& mag);
  void barCb(const BarMsg::ConstPtr& bar);
  void gpsCb(const GpsMsg::ConstPtr& gps);

  void executeCb(const GoalType& goal);
};
}  // namespace tobas_common_actions
