#pragma once

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tobas_ros2_tools/timer.hpp>
#include <tobas_node/node.hpp>
#include <tobas_msgs/Imu.h>
#include <tobas_msgs/MagneticField.h>

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

  using ImuMsg = tobas_msgs::Imu;
  using MagMsg = tobas_msgs::MagneticField;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  using ImuSubscriber = message_filters::Subscriber<ImuMsg>;
  using MagSubscriber = message_filters::Subscriber<MagMsg>;

public:
  explicit OrientationEstimatorRos(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  OrientationEstimator filter_;
  ImuMsg::ConstSharedPtr imu_;

  // RosParams
  bool do_bias_estimation_;
  bool do_adaptive_gain_;
  double gain_acc_;
  double gain_mag_;
  double bias_alpha_;

  // PubSub
  PublisherPtr<> orientation_pub_;
  ImuSubscriber imu_sub_;
  MagSubscriber mag_sub_;
  Synchronizer sync_;

  // Timer
  ros2::Timer check_topics_timer_;

  void getRosParams();
  void initializeFilter();

  void imuMagCb(const ImuMsg::ConstSharedPtr& imu, const MagMsg::ConstSharedPtr& mag);
  void checkTopicsTimerCb(const rclcpp::TimerEvent&);
};
}  // namespace orientation_estimation_complement
