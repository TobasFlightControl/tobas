#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <tobas_node/node.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tobas_debug_msgs_adapter/observer_feedback.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/geodetic_coordinates.hpp>
#include <tobas_msgs/srv/get_gnss_origin.hpp>
#include <tobas_msgs/srv/set_gnss_origin.hpp>
#include <tobas_msgs_adapter/gnss.hpp>
#include <tobas_msgs_adapter/imu.hpp>
#include <tobas_msgs_adapter/magnetic_field.hpp>
#include <tobas_msgs_adapter/odometry.hpp>

#include "./eskf.hpp"

class ErrorStateKalmanFilterNode : public tobas::BaseNode
{
  using self = ErrorStateKalmanFilterNode;
  using super = tobas::BaseNode;

  using ImuMsg = tobas_msgs::Imu;
  using MagMsg = tobas_msgs::MagneticField;
  using BaroMsg = tobas_msgs::msg::FluidPressure;
  using GnssMsg = tobas_msgs::Gnss;
  using OdomMsg = tobas_msgs::Odometry;
  using GnssOriginMsg = tobas_msgs::msg::GeodeticCoordinates;
  using FeedbackMsg = tobas_debug_msgs::ObserverFeedback;

  using GetOrigin = tobas_msgs::srv::GetGnssOrigin;
  using SetOrigin = tobas_msgs::srv::SetGnssOrigin;

  // Default parameters
  static constexpr char kDefaultFrameId[] = "unknown";  // 空文字だとTFが警告文を出すため適当なデフォルト値を設定
  static constexpr char kDefaultPositionSource[] = "gnss";
  static constexpr bool kDefaultAdaptiveGnssNoise = true;
  static constexpr bool kDefaultAdaptiveGravNoise = false;
  static constexpr bool kDefaultDoAccBiasEstimation = false;
  static constexpr bool kDefaultDoGyroBiasEstimation = true;
  static constexpr bool kDefaultDoMagHardBiasEstimation = false;
  static constexpr bool kDefaultDoMagSoftBiasEstimation = false;
  static constexpr bool kDefaultDoGravEstimation = true;

  // 標準偏差の初期値
  // 共分散行列は成長は遅いが収束は割と速いから，大きすぎるくらいで適当に決めてよい
  static constexpr double kInitPosStddev = 5.;      // [m]
  static constexpr double kInitVelStddev = 1.;      // [m/s]
  static constexpr double kInitRotStddev = M_PI_4;  // [rad]
  static constexpr double kInitMagStddev = 0.5;     // [-]

public:
  explicit ErrorStateKalmanFilterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum struct PositionSource
  {
    kGnss,
    kAirPressure,
  };

  // 固定値
  double lat_0_;  // 緯度のゼロ点 (Base Frame)
  double lon_0_;  // 経度のゼロ点 (Base Frame)
  double alt_0_;  // 高度のゼロ点 (Base Frame)

  Eigen::Vector3d pos_meas_;
  ImuMsg::ConstSharedPtr imu_raw_, imu_filt_;
  MagMsg::ConstSharedPtr mag_;
  BaroMsg::ConstSharedPtr baro_;
  GnssMsg::ConstSharedPtr gnss_;
  bool mag_ref_set_ = false;  // 地磁気の参照値が設定されているかどうか
  bool gnss_fix_ = false;
  double gnss_anomaly_score_ = 0.;

  eskf::ErrorStateKalmanFilter eskf_;

  // Static parameters
  std::string frame_id_;
  PositionSource pos_src_;
  bool adaptive_gnss_noise_;
  bool adaptive_grav_noise_;
  bool do_acc_bias_estimation_;
  bool do_gyro_bias_estimation_;
  bool do_mag_hard_bias_estimation_;
  bool do_mag_soft_bias_estimation_;
  bool do_grav_estimation_;
  Eigen::Vector3d imu_offset_;   // [m] ルートリンクに対するIMUの位置 (Local)
  Eigen::Vector3d baro_offset_;  // [m] ルートリンクに対する気圧センサの位置 (Local)
  Eigen::Vector3d gnss_offset_;  // [m] ルートリンクに対するGNSSレシーバの位置 (Local)

  // Dynamic parameters
  Eigen::Matrix3d fixed_acc_cov_ = Eigen::Matrix3d::Identity();       // [m^2/s^4]
  Eigen::Matrix3d fixed_gyro_cov_ = Eigen::Matrix3d::Identity();      // [rad^2/s^2]
  Eigen::Matrix3d fixed_mag_cov_ = Eigen::Matrix3d::Identity();       // [-]
  double fixed_head_var_ = 1.;                                        // [rad^2]
  double fixed_baro_alt_var_ = 1.;                                    // [m^2]
  Eigen::Matrix3d fixed_gnss_pos_cov_ = Eigen::Matrix3d::Identity();  // [m^2]
  Eigen::Matrix3d fixed_gnss_vel_cov_ = Eigen::Matrix3d::Identity();  // [m^2/s^2]
  Eigen::Matrix3d fixed_grav_cov_ = Eigen::Matrix3d::Identity();      // [m^2/s^4]
  double grav_stddev_min_ = 1.;                                       // [m/s^2]
  double grav_stddev_max_ = 1.;                                       // [m/s^2]
  double grav_stddev_rate_ = 0.;                                      // [-]

  // Publishers
  ros2::PublisherPtr<OdomMsg> odom_pub_;
  ros2::PublisherPtr<GnssOriginMsg> gnss_origin_pub_;
  ros2::PublisherPtr<FeedbackMsg> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<ImuMsg> imu_raw_sub_;
  ros2::SubscriberPtr<ImuMsg> imu_filt_sub_;
  ros2::SubscriberPtr<MagMsg> mag_sub_;
  ros2::SubscriberPtr<BaroMsg> baro_sub_;
  ros2::SubscriberPtr<GnssMsg> gnss_sub_;

  // Services
  ros2::ServiceServerPtr<GetOrigin> get_gnss_origin_ss_;
  ros2::ServiceServerPtr<SetOrigin> set_gnss_origin_ss_;

  // TF
  geometry_msgs::msg::TransformStamped tf_;
  tf2_ros::TransformBroadcaster tf_br_;

  void getStaticRosParams();
  bool setMagneticFieldRef(const Eigen::Vector3d& mag_W);
  void fillOdometryMsg(OdomMsg& odom) const;
  void publishGNSSOrigin() const;
  void publishFeedback(const std_msgs::msg::Header& header) const;
  double calcGravMeasNoiseStddev(const Eigen::Vector3d& acc) const;
  Eigen::Matrix3d calcGravMeasNoiseCov(const Eigen::Vector3d& acc) const;

  double initAccelBiasStddev() const;
  double initGyroBiasStddev() const;
  double initMagHardBiasStddev() const;
  double initMagSoftBiasStddev() const;
  double initGravBiasStddev() const;

  static const char* positionSourceEnumToString(const PositionSource& e);
  static bool positionSourceStringToEnum(const std::string& s, PositionSource& e);

  bool fixedAccMeasNoiseStddevCb(const double& p);
  bool fixedGyroMeasNoiseStddevCb(const double& p);
  bool fixedMagMeasNoiseStddevCb(const double& p);
  bool fixedHeadMeasNoiseStddevCb(const double& p);
  bool fixedBaroAltMeasNoiseStddevCb(const double& p);
  bool fixedGnssPosMeasNoiseStddevCb(const double& p);
  bool fixedGnssVelMeasNoiseStddevCb(const double& p);
  bool fixedGravMeasNoiseStddevCb(const double& p);
  bool adaptiveGravMeasNoiseStddevMinCb(const double& p);
  bool adaptiveGravMeasNoiseStddevMaxCb(const double& p);
  bool adaptiveGravMeasNoiseStddevRateCb(const double& p);
  bool accBiasProcNoiseDensityCb(const double& p);
  bool gyroBiasProcNoiseDensityCb(const double& p);
  bool magHardBiasProcNoiseDensityCb(const double& p);
  bool magSoftBiasProcNoiseDensityCb(const double& p);
  bool gravProcNoiseDensityCb(const double& ud_ug);

  void imuRawCb(const ImuMsg::ConstSharedPtr& imu_raw);
  void imuFiltCb(const ImuMsg::ConstSharedPtr& imu_filt);
  void magCb(const MagMsg::ConstSharedPtr& mag);
  void baroCb(const BaroMsg::ConstSharedPtr& baro);
  void gnssCb(const GnssMsg::ConstSharedPtr& gnss);

  void getGnssOriginCb(const GetOrigin::Request::ConstSharedPtr& req, const GetOrigin::Response::SharedPtr& res);
  void setGnssOriginCb(const SetOrigin::Request::ConstSharedPtr& req, const SetOrigin::Response::SharedPtr& res);
};
