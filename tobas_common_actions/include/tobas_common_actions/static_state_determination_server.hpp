#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <dh_std_tools/statistics.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/StaticStateDeterminationAction.h>

namespace tobas_common_actions
{
class StaticStateDeterminationServer : public tobas::BaseNode
{
  static constexpr char kActionName[] = "static_state_determination";

  // 中心極限定理によると，データ数が30以上なら多くの分布に対してサンプル平均の分布は近似的に正規分布になる．
  // よって，データ数がそれ以上ならば平均と分散の推定がより信頼できると一般的には考えられる． (GPT4)
  static constexpr uint32_t kMinimumImuCount = 100;
  static constexpr uint32_t kMinimumBarCount = 100;
  static constexpr uint32_t kMinimumGpsCount = 50;

  static constexpr double kStaticGyroThreshold = 0.5;              // [rad/s]
  static constexpr double kStaticAirPressureAltVarThreshold = 3.;  // [m]

  using super = tobas::BaseNode;

  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using BarMsg = sensor_msgs::FluidPressure;
  using GpsMsg = sensor_msgs::NavSatFix;
  using VelMsg = tobas_msgs::LinearVelocityWithCovariance;

  using ActionType = tobas_msgs::StaticStateDeterminationAction;
  using GoalType = tobas_msgs::StaticStateDeterminationGoalConstPtr;
  using ResultType = tobas_msgs::StaticStateDeterminationResult;
  using FeedbackType = tobas_msgs::StaticStateDeterminationFeedback;

public:
  explicit StaticStateDeterminationServer(ros::NodeHandle nh, ros::NodeHandle pnh);

private:
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
  geometry_msgs::Vector3 gyro_;
  dh_std::OnlineStatistics pressure_alt_stat_;

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
  bool isValidGoal(const GoalType& goal);
  bool isValidResult(const GoalType& goal);
  bool isStatic();

  void eventCb(const tobas_msgs::Event& event) override;
  void imuCb(const ImuMsg& imu);
  void magCb(const MagMsg& mag);
  void barCb(const BarMsg& bar);
  void gpsCb(const GpsMsg& gps);
  void velCb(const VelMsg& vel);

  void executeCb(const GoalType& goal);
};
}  // namespace tobas_common_actions
