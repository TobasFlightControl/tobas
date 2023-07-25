#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/NavSatFix.h>

#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/LinearVelocityWithCovariance.h>
#include <tobas_msgs/BaseState.h>
#include <tobas_common_actions/StaticStateDeterminationAction.h>
#include <state_estimation_eskf/StateEstimationEskfConfig.h>

#include "./eskf.hpp"

namespace state_estimation_eskf
{
class ErrorStateKalmanFilterRos : public tobas::BaseNode
{
  using super = tobas::BaseNode;

  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using BarMsg = sensor_msgs::FluidPressure;
  using GpsMsg = sensor_msgs::NavSatFix;
  using VelMsg = tobas_msgs::LinearVelocityWithCovariance;
  using StateMsg = tobas_msgs::BaseState;

  using ConfigType = state_estimation_eskf::StateEstimationEskfConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

  enum GeomagObserveMethod
  {
    RPY,
    YAW_ONLY,
  };

public:
  explicit ErrorStateKalmanFilterRos();

private:
  tobas::Drone drone_;

  // 固定値
  Eigen::Vector3d imu2gps_;  // IMUに対するGPSレシーバの位置
  double lat_0_;             // 緯度のゼロ点 (Base Frame)
  double lon_0_;             // 経度のゼロ点 (Base Frame)
  double alt_0_bar_;         // 気圧センサから求めた高度のゼロ点 (Base Frame)
  double alt_0_gps_;         // GPSから求めた高度のゼロ点 (Base Frame)
  Eigen::Quaterniond q_0_;   // 姿勢の初期値 (Base Frame)

  bool imu_received_;
  bool mag_received_;
  bool bar_received_;
  bool gps_received_;
  bool vel_received_;
  bool is_initialized_;
  ros::Time t_ready_;  // 全てのメッセージが確認され，ESKFが状態を更新し始める時刻
  ros::Time t_last_;
  StateMsg state_;  // 発行する状態
  double yaw_now_;
  double yaw_prev_;
  int yaw_jump_count_;  // ヨー角の回転回数

  Eigen::Vector3d a_m_;
  Eigen::Vector3d w_m_;
  Eigen::Vector3d mag_m_;
  Eigen::Vector3d pos_m_;
  Eigen::Vector3d vel_m_;
  Eigen::Matrix3d rot_acc_cov_;
  Eigen::Matrix3d rot_mag_cov_;

  ErrorStateKalmanFilter eskf_;

  // rosparams
  double gravity_;
  double ref_mag_north_;
  double ref_mag_east_;
  double ref_mag_down_;
  double gyro_noise_density_;  // rad/s/sqrt(hz)
  double gyro_random_walk_;    // rad/s^2/sqrt(hz)
  double acc_noise_density_;   // m/s^2/sqrt(hz)
  double acc_random_walk_;     // m/s^3/sqrt(hz)
  bool use_bar_;
  bool use_gps_;
  double gps_hor_pos_stddev_thr_;                         // [m]
  double gps_ver_pos_stddev_thr_;                         // [m]
  GeomagObserveMethod geomag_observe_method_;
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
  tobas_common_actions::StaticStateDeterminationResultConstPtr setZeroPositions();
  void updateBaseStateMsg(const ImuMsg& imu);
  bool isValidImuTimeGap(double dt);

  void eventCb(const tobas_msgs::Event& event) override;
  void imuCb(const ImuMsg& imu);
  void magCb(const MagMsg& mag);
  void barCb(const BarMsg& bar);
  void gpsCb(const GpsMsg& gps);
  void velCb(const VelMsg& vel);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace state_estimation_eskf
