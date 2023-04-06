#pragma once

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "./complementary_filter.hpp"

class ComplementaryFilterRos
{
  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  using ImuSubscriber = message_filters::Subscriber<ImuMsg>;
  using MagSubscriber = message_filters::Subscriber<MagMsg>;

public:
  ComplementaryFilterRos();

private:
  ros::NodeHandle nh_;

  ros::Publisher imu_pub_;

  ImuSubscriber imu_sub_;
  MagSubscriber mag_sub_;
  Synchronizer sync_;

  // rosparam
  bool do_bias_estimation_;
  bool do_adaptive_gain_;
  double gravity_;
  double gain_acc_;
  double gain_mag_;
  double bias_alpha_;
  double ref_mag_north_;
  double ref_mag_east_;
  double ref_mag_down_;

  ComplementaryFilter filter_;
  ros::Time time_prev_;
  bool is_initialized_;

  void getRosParams();
  void initializeFilter();
  void imuMagCb(const ImuMsg& imu, const MagMsg& mag);
};
