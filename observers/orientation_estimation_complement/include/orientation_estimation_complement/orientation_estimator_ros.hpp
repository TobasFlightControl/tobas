#pragma once

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>

#include "./orientation_estimator.hpp"

namespace orientation_estimation_complement
{
class OrientationEstimatorRos : public tobas::BaseNode
{
  // Constants
  static constexpr double kTimerPeriod = 5.;
  static constexpr uint32_t kQueueSize = 5;

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
  explicit OrientationEstimatorRos(ros::NodeHandle nh, ros::NodeHandle pnh);

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
  std::shared_ptr<ImuSubscriber> imu_sub_;
  std::shared_ptr<MagSubscriber> mag_sub_;
  std::shared_ptr<Synchronizer> sync_;

  // Timer
  dh_ros::Timer check_topics_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void initializeFilter();

  void eventCb(const tobas_msgs::Event& event) override;
  void imuMagCb(const ImuMsg& imu, const MagMsg& mag);
  void checkTopicsTimerCb(const ros::TimerEvent&);
};
}  // namespace orientation_estimation_complement
