#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/NavSatFix.h>

#include <dh_ros_tools/timer.hpp>
#include <dh_ros_tools/stopwatch.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/LinearVelocityWithCovariance.h>
#include <tobas_msgs/PoseTwist.h>

#include <state_estimation_eskf/ErrorStateKalmanFilterFeedback.h>
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
  using StateMsg = tobas_msgs::PoseTwist;
  using FeedbackMsg = state_estimation_eskf::ErrorStateKalmanFilterFeedback;

  using ConfigType = state_estimation_eskf::StateEstimationEskfConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

  enum GeomagObserveMethod
  {
    RPY,
    YAW_ONLY,
  };

public:
  explicit ErrorStateKalmanFilterRos(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  enum Stage
  {
    FIRST_IMU,
    WAIT_TOPICS,
    SET_FIRST_TIME,
    RUNNING,
  };

  tobas::Drone drone_;

  // 固定値
  Eigen::Vector3d imu2gps_;  // IMUに対するGPSレシーバの位置
  double lat_0_;             // 緯度のゼロ点 (Base Frame)
  double lon_0_;             // 経度のゼロ点 (Base Frame)
  double alt_0_gps_;         // GPS高度のゼロ点 (Base Frame)
  double alt_0_bar_;         // 気圧高度のゼロ点 (Base Frame)
  Eigen::Quaterniond q_0_;   // 姿勢の初期値 (Base Frame)

  Stage stage_ = FIRST_IMU;
  bool imu_received_ = false;
  bool mag_received_ = false;
  bool bar_received_ = false;
  bool gps_received_ = false;
  bool vel_received_ = false;
  ros::Time t_ready_;  // 全てのメッセージが確認され，ESKFが状態を更新し始める時刻
  ros::Time t_last_;
  double yaw_now_;
  double yaw_prev_;
  int yaw_jump_count_;  // ヨー角の回転回数

  Eigen::Vector3d a_m_;
  Eigen::Vector3d w_m_;
  Eigen::Vector3d mag_m_;
  Eigen::Vector3d pos_m_;
  Eigen::Vector3d vel_m_;
  Eigen::Matrix3d rot_acc_cov_ = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d rot_mag_cov_ = Eigen::Matrix3d::Zero();
  double acc_bias_noise_var_;
  double gyro_bias_noise_var_;

  ErrorStateKalmanFilter eskf_;

  // rosparams
  bool use_bar_;
  bool use_gps_;
  double gps_hor_pos_stddev_thr_;  // [m]
  double gps_ver_pos_stddev_thr_;  // [m]
  GeomagObserveMethod geomag_observe_method_;

  // PubSub
  ros::Publisher posevel_pub_;
  ros::Publisher feedback_pub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber mag_sub_;
  ros::Subscriber bar_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber vel_sub_;

  // Timer
  dh_ros::Timer check_topics_timer_;

  // Dynamic Reconfigure
  ConfigServer server_;

  // Other
  dh_ros::Stopwatch stopwatch_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady();
  void initialize();
  void setZeroPositions();
  StateMsg::ConstPtr makePoseVelMsg(const ImuMsg& imu);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void imuCb(const ImuMsg::ConstPtr& imu);
  void magCb(const MagMsg::ConstPtr& mag);
  void barCb(const BarMsg::ConstPtr& bar);
  void gpsCb(const GpsMsg::ConstPtr& gps);
  void velCb(const VelMsg::ConstPtr& vel);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace state_estimation_eskf
