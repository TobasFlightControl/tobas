#pragma once

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <tobas_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>

#include "./orientation_estimator.hpp"

namespace orientation_estimation_complement
{
class OrientationEstimatorRos : public tobas::BaseNode
{
  // Constants
  static constexpr double kTimerPeriod = 5.;
  static constexpr size_t kQueueSize = 5;

  // Default parameters
  static constexpr double kDefaultGainAcc = 0.01;
  static constexpr double kDefaultGainMag = 0.01;
  static constexpr double kDefaultBiasAlpha = 0.01;
  static constexpr bool kDefaultDoBiasEstimation = true;
  static constexpr bool kDefaultDoAdaptiveGain = false;

  using super = tobas::BaseNode;

  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  using ImuSubscriber = message_filters::Subscriber<ImuMsg>;
  using MagSubscriber = message_filters::Subscriber<MagMsg>;

public:
  explicit OrientationEstimatorRos(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  OrientationEstimator filter_;
  ros::Time time_prev_;
  bool is_initialized_;
  Eigen::Vector3d a_;
  Eigen::Vector3d w_;
  Eigen::Vector3d m_;

  // RosParams
  bool do_bias_estimation_;
  bool do_adaptive_gain_;
  double gain_acc_;
  double gain_mag_;
  double bias_alpha_;

  // PubSub
  ros::Publisher imu_pub_;
  ImuSubscriber imu_sub_;
  MagSubscriber mag_sub_;
  Synchronizer sync_;

  // Timer
  tobas_ros::Timer check_topics_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void initializeFilter();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void imuMagCb(const ImuMsg::ConstPtr& imu, const MagMsg::ConstPtr& mag);
  void checkTopicsTimerCb(const ros::TimerEvent&);
};
}  // namespace orientation_estimation_complement
