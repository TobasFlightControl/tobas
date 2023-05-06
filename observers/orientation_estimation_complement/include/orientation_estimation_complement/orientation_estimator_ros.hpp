#pragma once

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <dh_ros_tools/node.hpp>

#include "./orientation_estimator.hpp"

class OrientationEstimatorRos : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  using ImuSubscriber = message_filters::Subscriber<ImuMsg>;
  using MagSubscriber = message_filters::Subscriber<MagMsg>;

public:
  explicit OrientationEstimatorRos();

private:
  OrientationEstimator filter_;
  ros::Time time_prev_;
  bool is_initialized_;
  Eigen::Vector3d a_;
  Eigen::Vector3d w_;
  Eigen::Vector3d m_;

  // rosparams
  std::string drone_name_;
  bool do_bias_estimation_;
  bool do_adaptive_gain_;
  double gravity_;
  double gain_acc_;
  double gain_mag_;
  double bias_alpha_;
  double ref_mag_north_;
  double ref_mag_east_;
  double ref_mag_down_;

  // PubSub
  ros::Publisher imu_pub_;
  std::shared_ptr<ImuSubscriber> imu_sub_;
  std::shared_ptr<MagSubscriber> mag_sub_;
  std::shared_ptr<Synchronizer> sync_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;
  void createTimers() override;

  void initializeFilter();

  void imuMagCb(const ImuMsg& imu, const MagMsg& mag);
  void checkTopicsTimerCb(const ros::TimerEvent& event) override;
};
