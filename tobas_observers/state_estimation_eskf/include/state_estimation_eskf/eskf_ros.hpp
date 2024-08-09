#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <geometry_msgs/msg/TransformStamped.h>

#include <tobas_ros2_tools/timer.hpp>
#include <tobas_node/node.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_msgs/Imu.h>
#include <tobas_msgs/MagneticField.h>
#include <tobas_msgs/Gps.h>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/GetGnssOrigin.h>
#include <tobas_msgs/SetGnssOrigin.h>

#include <state_estimation_eskf/ErrorStateKalmanFilterFeedback.h>
#include <state_estimation_eskf/StateEstimationEskfConfig.h>

#include "./eskf.hpp"

namespace state_estimation_eskf
{
class ErrorStateKalmanFilterRos : public tobas::BaseNode
{
  static constexpr char kFeedbackTopic[] = "eskf_feedback";

  using self = ErrorStateKalmanFilterRos;
  using super = tobas::BaseNode;

  using ImuMsg = tobas_msgs::Imu;
  using MagMsg = tobas_msgs::MagneticField;
  using BarMsg = sensor_msgs::msg::FluidPressure;
  using GpsMsg = tobas_msgs::Gps;
  using OdomMsg = tobas_msgs::Odometry;
  using FeedbackMsg = state_estimation_eskf::ErrorStateKalmanFilterFeedback;

  using ConfigType = state_estimation_eskf::StateEstimationEskfConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit ErrorStateKalmanFilterRos(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;

  // 固定値
  double lat_0_;      // 緯度のゼロ点 (Base Frame)
  double lon_0_;      // 経度のゼロ点 (Base Frame)
  double alt_0_gps_;  // GPS高度のゼロ点 (Base Frame)
  double alt_0_bar_;  // 気圧高度のゼロ点 (Base Frame)
  double yaw_0_;      // ヨー角のゼロ点 (Base Frame)

  ImuMsg::ConstSharedPtr imu_, imu_filtered_;
  MagMsg::ConstSharedPtr mag_;
  BarMsg::ConstSharedPtr bar_;
  GpsMsg::ConstSharedPtr gps_;
  bool gps_fix_ = false;
  double gps_anormaly_score_ = 0.;

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
  Eigen::Vector3d imu_offset_;  // [m] ルートリンクに対するIMUの位置 (Local)
  Eigen::Vector3d bar_offset_;  // [m] ルートリンクに対する気圧センサの位置 (Local)
  Eigen::Vector3d gps_offset_;  // [m] ルートリンクに対するGPSレシーバの位置 (Local)

  // PubSub
  PublisherPtr<> odom_pub_;
  PublisherPtr<> feedback_pub_;
  SubscriberPtr<> imu_sub_;
  SubscriberPtr<> imu_filtered_sub_;
  SubscriberPtr<> mag_sub_;
  SubscriberPtr<> bar_sub_;
  SubscriberPtr<> gps_sub_;

  // Service
  rclcpp::ServiceServer get_gnss_origin_ss_;
  rclcpp::ServiceServer set_gnss_origin_ss_;

  // TF
  geometry_msgs::msg::TransformStamped tf_;
  tf2_ros::TransformBroadcaster tf_br_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams();
  OdomMsg::ConstSharedPtr makeOdometryMsg() const;

  void imuCb(const ImuMsg::ConstSharedPtr& imu);
  void imuFilteredCb(const ImuMsg::ConstSharedPtr& imu_filtered);
  void magCb(const MagMsg::ConstSharedPtr& mag);
  void barCb(const BarMsg::ConstSharedPtr& bar);
  void gpsCb(const GpsMsg::ConstSharedPtr& gps);

  bool getGnssOriginCb(tobas_msgs::GetGnssOriginRequest& req, tobas_msgs::GetGnssOriginResponse& res);
  bool setGnssOriginCb(tobas_msgs::SetGnssOriginRequest& req, tobas_msgs::SetGnssOriginResponse& res);


};
}  // namespace state_estimation_eskf
