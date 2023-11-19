#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>

#include <dh_std_tools/first_order_filter.hpp>
#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/Gps.h>
#include <tobas_msgs/Odometry.h>

#include <state_estimation_eskf/ErrorStateKalmanFilterFeedback.h>
#include <state_estimation_eskf/StateEstimationEskfConfig.h>

#include "./eskf.hpp"

namespace state_estimation_eskf
{
class ErrorStateKalmanFilterRos : public tobas::BaseNode
{
  using self = ErrorStateKalmanFilterRos;
  using super = tobas::BaseNode;

  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using BarMsg = sensor_msgs::FluidPressure;
  using GpsMsg = tobas_msgs::Gps;
  using OdomMsg = tobas_msgs::Odometry;
  using FeedbackMsg = state_estimation_eskf::ErrorStateKalmanFilterFeedback;

  using ConfigType = state_estimation_eskf::StateEstimationEskfConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit ErrorStateKalmanFilterRos(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  enum Stage
  {
    FIRST_IMU,
    WAIT_TOPICS,
    SET_FIRST_TIME,
    RUNNING,
  };

  // 固定値
  double lat_0_;            // 緯度のゼロ点 (Base Frame)
  double lon_0_;            // 経度のゼロ点 (Base Frame)
  double alt_0_gps_;        // GPS高度のゼロ点 (Base Frame)
  double alt_0_bar_;        // 気圧高度のゼロ点 (Base Frame)
  Eigen::Quaterniond q_0_;  // 姿勢の初期値 (Base Frame)

  Stage stage_ = FIRST_IMU;
  bool cov_converged_ = false;
  bool imu_received_ = false;
  bool mag_received_ = false;
  bool bar_received_ = false;
  bool gps_received_ = false;
  ros::Time t_last_;
  double yaw_now_;
  double yaw_prev_;
  int yaw_jump_count_;  // ヨー角の回転回数
  double gps_anormaly_score_ = 0.;

  Eigen::Vector3d acc_meas_;
  Eigen::Vector3d gyro_meas_;
  Eigen::Vector3d mag_meas_;
  Eigen::Vector3d pos_meas_;
  Eigen::Matrix3d grav_cov_ = Eigen::Matrix3d::Zero();
  double yaw_var_;
  double acc_bias_noise_var_;   // 加速度バイアスののプロセスノイズの分散
  double gyro_bias_noise_var_;  // ジャイロバイアスののプロセスノイズの分散
  double grav_noise_var_;       // 重力加速度のプロセスノイズの分散

  ErrorStateKalmanFilter eskf_;

  // rosparams
  bool use_bar_;
  bool use_gps_;
  bool do_acc_bias_estimation_;
  bool do_gyro_bias_estimation_;
  bool do_grav_estimation_;
  bool check_covariance_convergence_;
  Eigen::Vector3d imu_offset_;  // [m] ルートリンクに対するIMUの位置 (Local)
  Eigen::Vector3d bar_offset_;  // [m] ルートリンクに対する気圧センサの位置 (Local)
  Eigen::Vector3d gps_offset_;  // [m] ルートリンクに対するGPSレシーバの位置 (Local)
  double gps_hor_pos_stddev_thr_;  // [m]
  double gps_ver_pos_stddev_thr_;  // [m]

  // PubSub
  ros::Publisher odom_pub_;
  ros::Publisher feedback_pub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber mag_sub_;
  ros::Subscriber bar_sub_;
  ros::Subscriber gps_sub_;

  // Timer
  dh_ros::Timer check_topics_timer_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady() const;
  void initialize();
  void setZeroPositions();
  OdomMsg::ConstPtr makeOdometryMsg(const ImuMsg& imu);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void imuCb(const ImuMsg::ConstPtr& imu);
  void magCb(const MagMsg::ConstPtr& mag);
  void barCb(const BarMsg::ConstPtr& bar);
  void gpsCb(const GpsMsg::ConstPtr& gps);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace state_estimation_eskf
