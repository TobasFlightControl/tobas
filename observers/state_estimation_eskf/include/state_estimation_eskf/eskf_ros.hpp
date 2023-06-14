#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/NavSatFix.h>

#include <dh_ros_tools/node.hpp>
#include <dh_ros_tools/timer.hpp>

#include <tobas_msgs/LinearVelocityWithCovariance.h>
#include <tobas_msgs/BaseState.h>
#include <state_estimation_eskf/StateEstimationEskfConfig.h>

#include "./eskf.hpp"

namespace state_estimation_eskf
{
class ErrorStateKalmanFilterRos : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using BarMsg = sensor_msgs::FluidPressure;
  using GpsMsg = sensor_msgs::NavSatFix;
  using VelMsg = tobas_msgs::LinearVelocityWithCovariance;
  using StateMsg = tobas_msgs::BaseState;

  using ConfigType = state_estimation_eskf::StateEstimationEskfConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit ErrorStateKalmanFilterRos();

private:
  // 固定値
  double lat_0_;            // 緯度のゼロ点
  double lon_0_;            // 経度のゼロ点
  double alt_0_;            // 高度のゼロ点
  Eigen::Quaterniond q_0_;  // 姿勢の初期値

  bool is_initialized_;
  bool imu_received_;
  bool mag_received_;
  bool bar_received_;
  bool gps_received_;
  bool vel_received_;
  ros::Time t_ready_;  // 全てのメッセージが確認され，ESKFが状態を更新し始める時刻
  ros::Time t_last_;
  StateMsg state_;  // 発行する状態
  double yaw_now_;
  double yaw_prev_;
  int yaw_jump_count_;  // ヨー角の回転回数

  Eigen::Vector3d a_m_;
  Eigen::Vector3d w_m_;
  Eigen::Vector3d mag_m_;
  Eigen::Vector2d xy_m_;
  Eigen::Vector3d vel_m_;
  Eigen::Matrix3d rot_acc_cov_;
  Eigen::Matrix3d rot_mag_cov_;

  ErrorStateKalmanFilter eskf_;

  // rosparams
  double gravity_;
  double ref_mag_north_;
  double ref_mag_east_;
  double ref_mag_down_;
  double gyro_noise_density_;                             // rad/s/sqrt(hz)
  double gyro_random_walk_;                               // rad/s^2/sqrt(hz)
  double acc_noise_density_;                              // m/s^2/sqrt(hz)
  double acc_random_walk_;                                // m/s^3/sqrt(hz)
  bool use_gps_;
  double gps_pos_stddev_thr_;                             // [m]
  state_estimation_eskf::StateEstimationEskfConfig cfg_;  // 動的パラメータ

  // PubSub
  ros::Publisher posevel_pub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber mag_sub_;
  ros::Subscriber bar_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber vel_sub_;

  // Timer
  dh_ros::Timer check_topics_timer_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady();
  void initialize(const ros::Time& stamp);
  void setZeroPositions();
  void updateBaseStateMsg(const ros::Time& stamp);
  bool isValidImuTimeGap(double dt);

  void imuCb(const ImuMsg& imu);
  void magCb(const MagMsg& mag);
  void barCb(const BarMsg& bar);
  void gpsCb(const GpsMsg& gps);
  void velCb(const VelMsg& vel);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace state_estimation_eskf
