#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/PreArmCheckAction.h>

namespace tobas_common_actions
{
class PreArmCheckServer : public tobas::BaseNode
{
  static constexpr double kMeasureTime = 5.;              // [s]
  static constexpr double kGyroNormThreshold = M_PI / 6;  // [rad/s]
  static constexpr double kAccelErrorThreshold = 1e+100;  // [m/s^2]  // FIXME: SIMで接地時に暴れる
  static constexpr double kAirAltStddevThreshold = 1e+100;  // [m]  // TODO
  static constexpr double kGpsAltStddevThreshold = 5.;      // [m]
  static constexpr double kGpsPosCovStddevThreshold = 5.;   // [m]

  // 状態の標準偏差の閾値
  static constexpr double kHorPosStddevThreshold = 1.0;     // [m]
  static constexpr double kVerPosStddevThreshold = 2.0;     // [m]
  static constexpr double kVelStddevThreshold = 0.3;        // [m/s]
  static constexpr double kRotStddevThreshold = M_PI / 24;  // [rad]

  using self = PreArmCheckServer;
  using super = tobas::BaseNode;

  using BatMsg = tobas_msgs::Battery;
  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using BarMsg = sensor_msgs::FluidPressure;
  using GpsMsg = tobas_msgs::Gps;

  using ActionType = tobas_msgs::PreArmCheckAction;
  using GoalType = tobas_msgs::PreArmCheckGoal;
  using ResultType = tobas_msgs::PreArmCheckResult;
  using FeedbackType = tobas_msgs::PreArmCheckFeedback;

public:
  explicit PreArmCheckServer(
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

  void executeCb(const GoalType::ConstPtr& goal);
};
}  // namespace tobas_common_actions
