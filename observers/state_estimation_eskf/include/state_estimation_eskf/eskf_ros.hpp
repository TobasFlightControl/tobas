#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/NavSatFix.h>

#include <multirotor_msgs/LinearVelocityWithCovarianceStamped.h>
#include <multirotor_msgs/PoseVelStamped.h>

#include "./eskf.hpp"

class ErrorStateKalmanFilterRos
{
  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using BarMsg = sensor_msgs::FluidPressure;
  using GpsMsg = sensor_msgs::NavSatFix;
  using VelMsg = multirotor_msgs::LinearVelocityWithCovarianceStamped;
  using StateMsg = multirotor_msgs::PoseVelStamped;

public:
  ErrorStateKalmanFilterRos(ros::NodeHandle& nh);

private:
  const double gyro_noise_density_;  // rad/s/sqrt(hz)
  const double gyro_random_walk_;    // rad/s^2/sqrt(hz)
  const double acc_noise_density_;   // m/s^2/sqrt(hz)
  const double acc_random_walk_;     // m/s^3/sqrt(hz)
  const Eigen::Vector3d ref_mag_;

  bool is_ready_;
  ros::Time t_last_;
  double lat_0_;  // 緯度のゼロ点
  double lon_0_;  // 経度のゼロ点
  double alt_0_;  // 高度のゼロ点
  bool imu_subscribed_;
  bool mag_subscribed_;
  bool bar_subscribed_;
  bool gps_subscribed_;
  bool vel_subscribed_;
  ImuMsg imu_;      // IMUの観測値
  MagMsg mag_;      // 磁気センサの観測値
  BarMsg bar_;      // 気圧センサの観測値
  GpsMsg gps_;      // GPS位置の観測値
  VelMsg vel_;      // GPS速度の観測値
  StateMsg state_;  // 発行する状態

  Eigen::Vector3d a_m_;
  Eigen::Vector3d w_m_;
  Eigen::Quaterniond q_m_;
  Eigen::Vector2d xy_m_;
  Eigen::Vector3d v_m_;

  ErrorStateKalmanFilter eskf_;

  ros::Publisher posevel_pub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber mag_sub_;
  ros::Subscriber bar_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber vel_sub_;

  bool allMsgReceived();
  void initialize();
  void setZeroPositions();
  void updatePoseVelMsg();

  void imuCb(const ImuMsg& msg);
  void magCb(const MagMsg& msg);
  void barCb(const BarMsg& msg);
  void gpsCb(const GpsMsg& msg);
  void velCb(const VelMsg& msg);

  static lTime getNow();
};
