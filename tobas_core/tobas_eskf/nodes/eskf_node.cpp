#include <tf2_ros/transform_broadcaster.h>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_geomag/core.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/gnss.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_time_tools/util.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tobas_debug_msgs_adapter/observer_feedback.hpp>
#include <tobas_kdl_msgs_adapter/frame_with_covariance_stamped.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/geodetic_coordinates.hpp>
#include <tobas_msgs/srv/get_gnss_origin.hpp>
#include <tobas_msgs/srv/set_gnss_origin.hpp>
#include <tobas_msgs_adapter/gnss.hpp>
#include <tobas_msgs_adapter/imu.hpp>
#include <tobas_msgs_adapter/magnetic_field.hpp>
#include <tobas_msgs_adapter/odometry.hpp>

#include "tobas_eskf/eskf.hpp"
#include "tobas_eskf/util.hpp"

using namespace Eigen;

class ErrorStateKalmanFilterNode : public tobas::BaseNode
{
  using self = ErrorStateKalmanFilterNode;
  using super = tobas::BaseNode;

  using ImuMsg = tobas_msgs::Imu;
  using MagMsg = tobas_msgs::MagneticField;
  using BaroMsg = tobas_msgs::msg::FluidPressure;
  using GnssMsg = tobas_msgs::Gnss;
  using PoseMsg = tobas_kdl_msgs::FrameWithCovarianceStamped;
  using OdomMsg = tobas_msgs::Odometry;
  using MagRefMsg = tobas_msgs::MagneticField;
  using GnssOriginMsg = tobas_msgs::msg::GeodeticCoordinates;
  using FeedbackMsg = tobas_debug_msgs::ObserverFeedback;

  using GetOrigin = tobas_msgs::srv::GetGnssOrigin;
  using SetOrigin = tobas_msgs::srv::SetGnssOrigin;

  // Default parameters
  static constexpr char kDefaultFrameId[] = "unknown";  // 空文字だとTFが警告文を出すため適当なデフォルト値を設定
  static constexpr bool kDefaultUseMagnetometer = true;
  static constexpr bool kDefaultUseBarometer = false;
  static constexpr bool kDefaultUseGnss = true;
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
  // 固定値
  double lat_0_;  // 緯度のゼロ点 (Base Frame)
  double lon_0_;  // 経度のゼロ点 (Base Frame)
  double alt_0_;  // 高度のゼロ点 (Base Frame)

  Vector3d pos_meas_;
  Matrix6d gnss_cov_ = Matrix6d::Zero();
  ImuMsg::ConstSharedPtr imu_raw_, imu_filt_;
  MagMsg::ConstSharedPtr mag_;
  BaroMsg::ConstSharedPtr baro_;
  GnssMsg::ConstSharedPtr gnss_;
  PoseMsg::ConstSharedPtr pose_;
  bool mag_ref_set_ = false;  // 地磁気の参照値が設定されているかどうか
  bool gnss_fix_ = false;
  double gnss_anomaly_score_ = 0.;

  eskf::ErrorStateKalmanFilter eskf_;

  // Static parameters
  std::string frame_id_;
  bool use_mag_;
  bool use_baro_;
  bool use_gnss_;
  bool adaptive_gnss_noise_;
  bool adaptive_grav_noise_;
  bool do_acc_bias_estimation_;
  bool do_gyro_bias_estimation_;
  bool do_mag_hard_bias_estimation_;
  bool do_mag_soft_bias_estimation_;
  bool do_grav_estimation_;
  Vector3d imu_offset_;   // [m] ルートリンクに対するIMUの位置 (Local)
  Vector3d baro_offset_;  // [m] ルートリンクに対する気圧センサの位置 (Local)
  Vector3d gnss_offset_;  // [m] ルートリンクに対するGNSSレシーバの位置 (Local)

  // Dynamic parameters
  Matrix3d fixed_acc_cov_ = Matrix3d::Identity();       // [m^2/s^4]
  Matrix3d fixed_gyro_cov_ = Matrix3d::Identity();      // [rad^2/s^2]
  Matrix3d fixed_mag_cov_ = Matrix3d::Identity();       // [-]
  double fixed_head_var_ = 1.;                          // [rad^2]
  double fixed_baro_alt_var_ = 1.;                      // [m^2]
  Matrix3d fixed_gnss_pos_cov_ = Matrix3d::Identity();  // [m^2]
  Matrix3d fixed_gnss_vel_cov_ = Matrix3d::Identity();  // [m^2/s^2]
  Matrix3d fixed_grav_cov_ = Matrix3d::Identity();      // [m^2/s^4]
  double grav_stddev_min_ = 1.;                         // [m/s^2]
  double grav_stddev_max_ = 1.;                         // [m/s^2]
  double grav_stddev_rate_ = 0.;                        // [-]

  // Publishers
  ros2::PublisherPtr<OdomMsg> odom_pub_;
  ros2::PublisherPtr<MagRefMsg> mag_ref_pub_;
  ros2::PublisherPtr<GnssOriginMsg> gnss_origin_pub_;
  ros2::PublisherPtr<FeedbackMsg> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<ImuMsg> imu_raw_sub_;
  ros2::SubscriberPtr<ImuMsg> imu_filt_sub_;
  ros2::SubscriberPtr<MagMsg> mag_sub_;
  ros2::SubscriberPtr<BaroMsg> baro_sub_;
  ros2::SubscriberPtr<GnssMsg> gnss_sub_;
  ros2::SubscriberPtr<PoseMsg> pose_sub_;

  // Services
  ros2::ServiceServerPtr<GetOrigin> get_gnss_origin_ss_;
  ros2::ServiceServerPtr<SetOrigin> set_gnss_origin_ss_;

  // TF
  geometry_msgs::msg::TransformStamped tf_;
  tf2_ros::TransformBroadcaster tf_br_;

  void getStaticRosParams();
  bool setMagneticFieldRef(const Vector3d& mag_W);
  void fillOdometryMsg(OdomMsg& odom) const;
  void publishMagRef(const Vector3d& mag_W) const;
  void publishGnssOrigin(double lat, double lon, double alt) const;
  void publishFeedback(const std_msgs::msg::Header& header) const;
  double calcGravMeasNoiseStddev(const Vector3d& acc) const;
  Matrix3d calcGravMeasNoiseCov(const Vector3d& acc) const;

  double initAccelBiasStddev() const;
  double initGyroBiasStddev() const;
  double initMagHardBiasStddev() const;
  double initMagSoftBiasStddev() const;
  double initGravBiasStddev() const;

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
  void poseCb(const PoseMsg::ConstSharedPtr& msg);

  void getGnssOriginCb(const GetOrigin::Request::ConstSharedPtr& req, const GetOrigin::Response::SharedPtr& res);
  void setGnssOriginCb(const SetOrigin::Request::ConstSharedPtr& req, const SetOrigin::Response::SharedPtr& res);
};

ErrorStateKalmanFilterNode::ErrorStateKalmanFilterNode(const rclcpp::NodeOptions& options)
  : super(tobas::node::kObserver, options), tf_br_(this)
{
  getStaticRosParams();

  // Fill the static part of the transform message
  tf_.header.frame_id = tobas::kWorldFrame;
  tf_.child_frame_id = frame_id_;

  // Register dynamic parameters
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_ACC_NOISE
  addDynamicDoubleParam("acc_meas_noise_stddev", &self::fixedAccMeasNoiseStddevCb, this, 0.05, 20, 1, 20, " m/s^2");
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_GYR_NOISE
  addDynamicDoubleParam("gyro_meas_noise_stddev", &self::fixedGyroMeasNoiseStddevCb, this, 0.005, 20, 1, 20, " rad/s");
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_MAG_NOISE
  addDynamicDoubleParam("mag_meas_noise_stddev", &self::fixedMagMeasNoiseStddevCb, this, 5., 1, 1, 20, " uT");
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_HEAD_NOISE
  addDynamicDoubleParam("head_meas_noise_stddev", &self::fixedHeadMeasNoiseStddevCb, this, 0.05, 6, 1, 20, " rad");
  // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_BARO_NOISE
  addDynamicDoubleParam("baro_alt_meas_noise_stddev", &self::fixedBaroAltMeasNoiseStddevCb, this, 0.5, 7, 1, 30, " m");
  if (!adaptive_gnss_noise_) {
    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_GPS_P_NOISE
    addDynamicDoubleParam(
      "gnss_pos_meas_noise_stddev", &self::fixedGnssPosMeasNoiseStddevCb, this, 0.1, 5, 1, 100, " m");
    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_GPS_V_NOISE
    addDynamicDoubleParam(
      "gnss_vel_meas_noise_stddev", &self::fixedGnssVelMeasNoiseStddevCb, this, 0.1, 3, 1, 50, " m/s");
  }
  if (adaptive_grav_noise_) {
    addDynamicDoubleParam(
      "grav_meas_noise_stddev_min", &self::adaptiveGravMeasNoiseStddevMinCb, this, 0.01, 1, 0, 100, " m/s^2");
    addDynamicDoubleParam(
      "grav_meas_noise_stddev_max", &self::adaptiveGravMeasNoiseStddevMaxCb, this, 1., 20, 10, 100, " m/s^2");
    addDynamicDoubleParam("grav_meas_noise_stddev_rate", &self::adaptiveGravMeasNoiseStddevRateCb, this, 5., 20, 0, 100);
  }
  else {
    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_GRAV_NOISE
    addDynamicDoubleParam("grav_meas_noise_stddev", &self::fixedGravMeasNoiseStddevCb, this, 0.1, 3, 1, 20, " g");
  }
  if (do_acc_bias_estimation_) {
    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_ACC_B_NOISE
    addDynamicDoubleParam(
      "acc_bias_proc_noise_density", &self::accBiasProcNoiseDensityCb, this, 1., 15, 0, 50, " ug/s/√Hz");
  }
  if (do_gyro_bias_estimation_) {
    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_GYR_B_NOISE
    addDynamicDoubleParam(
      "gyro_bias_proc_noise_density", &self::gyroBiasProcNoiseDensityCb, this, 1., 3, 0, 30, " mdps/s/√Hz");
  }
  if (do_mag_hard_bias_estimation_) {
    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_MAG_B_NOISE
    addDynamicDoubleParam(
      "mag_hard_bias_proc_noise_density", &self::magHardBiasProcNoiseDensityCb, this, 0.1, 5, 0, 100, " nT/s/√Hz");
  }
  if (do_mag_soft_bias_estimation_) {
    // cf. https://docs.px4.io/main/en/advanced_config/parameter_reference.html#EKF2_MAG_B_NOISE
    addDynamicDoubleParam(
      "mag_soft_bias_proc_noise_density", &self::magSoftBiasProcNoiseDensityCb, this, 0.1, 5, 0, 100, " nT/s/√Hz");
  }
  if (do_grav_estimation_) {
    addDynamicDoubleParam(
      "grav_noise_proc_noise_density", &self::gravProcNoiseDensityCb, this, 1., 15, 0, 50, " ug/s/√Hz");
  }

  // Register publishers
  odom_pub_ = createPublisher<OdomMsg>(tobas::kOdometryTopic);
  mag_ref_pub_ = createPublisher<MagRefMsg>(tobas::kMagRefTopic, true, true);
  gnss_origin_pub_ = createPublisher<GnssOriginMsg>(tobas::kGnssOriginTopic, true, true);
  feedback_pub_ = createPublisher<FeedbackMsg>(tobas::kObsvFeedbackTopic);

  // Register subscribers
  imu_raw_sub_ = createSubscriber(tobas::kImuRawTopic, &self::imuRawCb, this);
  imu_filt_sub_ = createSubscriber(tobas::kImuFiltTopic, &self::imuFiltCb, this);
  if (use_mag_) {
    mag_sub_ = createSubscriber(tobas::kMagTopic, &self::magCb, this);
  }
  if (use_baro_) {
    baro_sub_ = createSubscriber(tobas::kAirPressureTopic, &self::baroCb, this);
  }
  if (use_gnss_) {
    gnss_sub_ = createSubscriber(tobas::kGnssTopic, &self::gnssCb, this);
  }
  pose_sub_ = createSubscriber(tobas::kExternalPoseTopic, &self::poseCb, this);

  // Register service servers
  get_gnss_origin_ss_ = createService<GetOrigin>(tobas::kGetGnssOriginSrv, &self::getGnssOriginCb, this);
  set_gnss_origin_ss_ = createService<SetOrigin>(tobas::kSetGnssOriginSrv, &self::setGnssOriginCb, this);
}

void ErrorStateKalmanFilterNode::getStaticRosParams()
{
  frame_id_ = getStringParam("frame_id", kDefaultFrameId);

  use_mag_ = getBoolParam("use_magnetometer", kDefaultUseMagnetometer);
  use_baro_ = getBoolParam("use_barometer", kDefaultUseBarometer);
  use_gnss_ = getBoolParam("use_gnss", kDefaultUseGnss);

  adaptive_gnss_noise_ = getBoolParam("adaptive_gnss_noise", kDefaultAdaptiveGnssNoise);
  adaptive_grav_noise_ = getBoolParam("adaptive_grav_noise", kDefaultAdaptiveGravNoise);

  do_acc_bias_estimation_ = getBoolParam("do_acc_bias_estimation", kDefaultDoAccBiasEstimation);
  do_gyro_bias_estimation_ = getBoolParam("do_gyro_bias_estimation", kDefaultDoGyroBiasEstimation);
  do_mag_hard_bias_estimation_ = getBoolParam("do_mag_hard_bias_estimation", kDefaultDoMagHardBiasEstimation);
  do_mag_soft_bias_estimation_ = getBoolParam("do_mag_soft_bias_estimation", kDefaultDoMagSoftBiasEstimation);
  do_grav_estimation_ = getBoolParam("do_gravity_estimation", kDefaultDoGravEstimation);

  const auto imu_offset = getDoubleArrayParam("imu_offset", std::vector<double>(3, 0.));
  const auto baro_offset = getDoubleArrayParam("barometer_offset", std::vector<double>(3, 0.));
  const auto gnss_offset = getDoubleArrayParam("gnss_offset", std::vector<double>(3, 0.));
  imu_offset_ = Map<const Vector3d>(imu_offset.data());
  baro_offset_ = Map<const Vector3d>(baro_offset.data());
  gnss_offset_ = Map<const Vector3d>(gnss_offset.data());
}

bool ErrorStateKalmanFilterNode::setMagneticFieldRef(const Vector3d& mag_W)
{
  // 地磁気の参照値を設定
  if (!eskf_.setMagneticFieldRef(mag_W)) {
    TOBAS_ERROR("Failed to set reference magnetic field.");
    return false;
  }

  // 地磁気のバイアスを初期化
  eskf_.initializeMagHardBias(Vector3d::Zero(), Vector3d::Constant(math::sqr(initMagHardBiasStddev())).asDiagonal());
  eskf_.initializeMagSoftBias(Matrix3d::Identity(), Vector6d::Constant(math::sqr(initMagSoftBiasStddev())).asDiagonal());

  // 地磁気を受け取っていればヨーを初期化
  // でないとヨーの誤差が大きすぎる場合にロールピッチまでフィードバックの影響を受けてしまう
  if (mag_) {
    // 現在のRPYを取得
    double old_roll, old_pitch, old_yaw;
    const auto R_W_B = eskf_.getQuaternion();
    tbs::eulerFromQuaternion(R_W_B.x(), R_W_B.y(), R_W_B.z(), R_W_B.w(), old_roll, old_pitch, old_yaw);

    // 地磁気をヨー角のみ機体と一致し，XY軸が地面と平行な地上座標系Gに移す．
    const AngleAxisd R_W_G(old_yaw, Vector3d::UnitZ());
    const auto mag_G = R_W_G.inverse() * (R_W_B * mag_->mag.data);  // 後ろから計算することで計算量を削減
    const auto& mx = mag_G.x();
    const auto& my = mag_G.y();

    // 新しい参照に基づくヨーを計算
    const auto yaw_ref = atan2(mag_W.y(), mag_W.x());
    const auto new_yaw = algo::wrapPi(yaw_ref - atan2(my, mx));

    // ヨーのみ修正したクオータニオンを計算
    const auto new_q = eigen::quaternionFromRPY(old_roll, old_pitch, new_yaw);

    // 姿勢の共分散のヨー成分を修正
    auto rot_cov = eskf_.getRotationCovariance();
    rot_cov.row(2).setZero();
    rot_cov.col(2).setZero();
    rot_cov(2, 2) = math::sqr(kInitRotStddev);

    // 姿勢を初期化
    if (!eskf_.initializeQuaternion(new_q, rot_cov)) {
      TOBAS_ERROR("Failed to initialize orientation.");
      return false;
    }
  }

  // 地磁気の参照値を発行
  publishMagRef(mag_W);

  TOBAS_INFO("Reference magnetic field is set to be: ", mag_W.transpose());
  mag_ref_set_ = true;

  return true;
}

void ErrorStateKalmanFilterNode::fillOdometryMsg(OdomMsg& odom) const
{
  const Vector3d W_Pos_WI = eskf_.getPosition();
  const Vector3d W_Vel_WI = eskf_.getVelocity();
  const Quaterniond W_Rot_B = eskf_.getQuaternion();
  const Quaterniond B_Rot_W = W_Rot_B.conjugate();
  const Vector3d B_grav = B_Rot_W * Vector3d(0, 0, -eskf_.getGravity());
  const Vector3d B_Acc = imu_filt_->accel.data - eskf_.getAccelBias() + B_grav;  // 重力を除いた加速度
  const Vector3d B_Gyro = imu_filt_->gyro.data - eskf_.getGyroBias();

  // Header
  odom.header.stamp = imu_raw_->header.stamp;
  odom.header.frame_id = tobas::kWorldFrame;

  // Position (Global): IMU frame -> Base frame
  odom.frame.p.data = W_Pos_WI - W_Rot_B * imu_offset_;
  odom.position_covariance = eskf_.getPositionCovariance();

  // Linear velocity (Local): IMU frame -> Base frame
  odom.twist.vel.data = B_Rot_W * W_Vel_WI - B_Gyro.cross(imu_offset_);
  odom.velocity_covariance = B_Rot_W * eskf_.getVelocityCovariance() * W_Rot_B;

  // Orientation (Global)
  odom.frame.M.data = W_Rot_B.toRotationMatrix();
  odom.orientation_covariance = eskf_.getRotationCovariance();

  // Angular velocity (Local)
  odom.twist.rot.data = B_Gyro;
  odom.gyro_covariance = fixed_gyro_cov_ + eskf_.getGyroBiasCovariance();

  // Linear acceleration (Local)
  odom.accel.linear.data = B_Acc;

  // Angular acceleration (Local)
  odom.accel.angular = imu_filt_->dgyro;
}

void ErrorStateKalmanFilterNode::publishMagRef(const Vector3d& mag_W) const
{
  auto msg = std::make_unique<MagRefMsg>();

  msg->header.stamp = now();
  msg->mag = mag_W;

  mag_ref_pub_->publish(std::move(msg));
}

void ErrorStateKalmanFilterNode::publishGnssOrigin(double lat, double lon, double alt) const
{
  auto msg = std::make_unique<GnssOriginMsg>();

  msg->header.stamp = now();
  msg->latitude = lat;
  msg->longitude = lon;
  msg->altitude = alt;

  gnss_origin_pub_->publish(std::move(msg));
}

void ErrorStateKalmanFilterNode::publishFeedback(const std_msgs::msg::Header& header) const
{
  // Create
  auto feedback = std::make_unique<FeedbackMsg>();

  // Header
  feedback->header = header;

  // Data
  feedback->position = eskf_.getPosition();
  feedback->velocity = eskf_.getVelocity();
  feedback->hamilton = eskf_.getHamilton();
  feedback->accel_bias = eskf_.getAccelBias();
  feedback->gyro_bias = eskf_.getGyroBias();
  feedback->mag_hard_bias = eskf_.getMagHardBias();
  feedback->mag_soft_bias = eskf_.getMagSoftBias();
  feedback->gravity = eskf_.getGravity();

  // Variance
  feedback->position_cov = eskf_.getPositionCovariance();
  feedback->velocity_cov = eskf_.getVelocityCovariance();
  feedback->rotation_cov = eskf_.getRotationCovariance();
  feedback->accel_bias_cov = eskf_.getAccelBiasCovariance();
  feedback->gyro_bias_cov = eskf_.getGyroBiasCovariance();
  feedback->mag_hard_bias_cov = eskf_.getMagHardBiasCovariance();
  feedback->mag_soft_bias_cov = eskf_.getMagSoftBiasCovariance();
  feedback->gravity_var = eskf_.getGravityVariance();

  // Other
  feedback->gnss_anomaly_score = gnss_anomaly_score_;

  // Publish
  feedback_pub_->publish(std::move(feedback));
}

double ErrorStateKalmanFilterNode::calcGravMeasNoiseStddev(const Vector3d& acc) const
{
  // 加速度のL2ノルムから重力方向の観測の不確かさを決める．
  // 加速度の大きさと重力加速度との誤差が大きいほど重力以外の加速度が生じているため加速度による姿勢の観測が不確かだと考えるのは直感的だが，
  // その誤差は正規分布に従うわけではなく一様に確かでもないため，誤差をそのまま標準偏差とすることには何の根拠もない．
  // 実際，重力方向の分散を下げると，並進移動時に進行方向への加速度により実際よりも大きく傾いていると判断され，
  // 制御器が姿勢を戻そうとし，並進方向の加速度の追従が遅れ，位置制御が振動するという因果関係がある．
  // 動的加速度が陽にモデルに含まれていない以上，その不確かさの決定はヒューリスティックにならざるを得ない．
  // 実用的には動作時の追従遅れと静止時の収束速度のトレードオフを考慮して決定するしかないだろう．
  const auto acc_norm_diff = std::abs(acc.norm() - eskf_.getGravity());  // TODO: モデルから推定した動的加速度を考慮
  const auto grav_stddev = grav_stddev_min_ + grav_stddev_rate_ * acc_norm_diff;  // TODO: 他のプロファイルを検討
  return std::min(grav_stddev, grav_stddev_max_);
}

Matrix3d ErrorStateKalmanFilterNode::calcGravMeasNoiseCov(const Vector3d& acc) const
{
  const auto grav_stddev = calcGravMeasNoiseStddev(acc);
  const auto grav_var = math::sqr(grav_stddev);
  return Vector3d::Constant(grav_var).asDiagonal();
}

double ErrorStateKalmanFilterNode::initAccelBiasStddev() const
{
  return do_acc_bias_estimation_ ? 1. : 0.;
}

double ErrorStateKalmanFilterNode::initGyroBiasStddev() const
{
  return do_gyro_bias_estimation_ ? 0.1 : 0.;
}

double ErrorStateKalmanFilterNode::initMagHardBiasStddev() const
{
  return do_mag_hard_bias_estimation_ ? 0.1 : 0.;
}

double ErrorStateKalmanFilterNode::initMagSoftBiasStddev() const
{
  return do_mag_soft_bias_estimation_ ? 0.1 : 0.;
}

double ErrorStateKalmanFilterNode::initGravBiasStddev() const
{
  return do_grav_estimation_ ? 0.1 : 0.;
}

bool ErrorStateKalmanFilterNode::fixedAccMeasNoiseStddevCb(const double& p)
{
  const auto acc_stddev = p;  // [m/s^2]
  const auto acc_var = math::sqr(acc_stddev);
  fixed_acc_cov_.diagonal().fill(acc_var);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedGyroMeasNoiseStddevCb(const double& p)
{
  const auto gyro_stddev = p;  // [rad/s]
  const auto gyro_var = math::sqr(gyro_stddev);
  fixed_gyro_cov_.diagonal().fill(gyro_var);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedMagMeasNoiseStddevCb(const double& p)
{
  const auto mag_stddev = p * 1e-2 / tbs::kGeomagScale;  // [-]
  const auto mag_var = math::sqr(mag_stddev);
  fixed_mag_cov_.diagonal().fill(mag_var);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedHeadMeasNoiseStddevCb(const double& p)
{
  const auto head_stddev = p;  // [rad]
  fixed_head_var_ = math::sqr(head_stddev);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedBaroAltMeasNoiseStddevCb(const double& p)
{
  const auto baro_alt_stddev = p;  // [m]
  fixed_baro_alt_var_ = math::sqr(baro_alt_stddev);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedGnssPosMeasNoiseStddevCb(const double& p)
{
  assert(!adaptive_gnss_noise_);

  const auto gnss_pos_stddev = p;  // [m]
  const auto gnss_pos_var = math::sqr(gnss_pos_stddev);
  fixed_gnss_pos_cov_.diagonal().fill(gnss_pos_var);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedGnssVelMeasNoiseStddevCb(const double& p)
{
  assert(!adaptive_gnss_noise_);

  const auto gnss_vel_stddev = p;  // [m/s]
  const auto gnss_vel_var = math::sqr(gnss_vel_stddev);
  fixed_gnss_vel_cov_.diagonal().fill(gnss_vel_var);

  return true;
}

bool ErrorStateKalmanFilterNode::fixedGravMeasNoiseStddevCb(const double& p)
{
  assert(!adaptive_grav_noise_);

  const auto grav_stddev = p * tbs::kGravity;  // [m/s^2]
  const auto grav_var = math::sqr(grav_stddev);
  fixed_grav_cov_.diagonal().fill(grav_var);

  return true;
}

bool ErrorStateKalmanFilterNode::adaptiveGravMeasNoiseStddevMinCb(const double& p)
{
  assert(adaptive_grav_noise_);
  grav_stddev_min_ = p;
  return true;
}

bool ErrorStateKalmanFilterNode::adaptiveGravMeasNoiseStddevMaxCb(const double& p)
{
  assert(adaptive_grav_noise_);
  grav_stddev_max_ = p;
  return true;
}

bool ErrorStateKalmanFilterNode::adaptiveGravMeasNoiseStddevRateCb(const double& p)
{
  assert(adaptive_grav_noise_);
  grav_stddev_rate_ = p;
  return true;
}

bool ErrorStateKalmanFilterNode::accBiasProcNoiseDensityCb(const double& p)
{
  assert(do_acc_bias_estimation_);

  const auto nd = p * 1e-6 * tbs::kGravity;  // ug/s/√Hz -> m/s^3/√Hz
  return eskf_.setAccBiasProcNoiseDensity(nd);
}

bool ErrorStateKalmanFilterNode::gyroBiasProcNoiseDensityCb(const double& p)
{
  assert(do_gyro_bias_estimation_);

  const auto nd = p * 1e-3 * tbs::kDeg2Rad;  // mdps/s/√Hz -> rad/s^2/√Hz
  return eskf_.setGyroBiasProcNoiseDensity(nd);
}

bool ErrorStateKalmanFilterNode::magHardBiasProcNoiseDensityCb(const double& p)
{
  assert(do_mag_hard_bias_estimation_);

  const auto nd = p * 1e-5 / tbs::kGeomagScale;  // nT/s/√Hz -> /s/√Hz
  return eskf_.setMagHardBiasProcNoiseDensity(nd);
}

bool ErrorStateKalmanFilterNode::magSoftBiasProcNoiseDensityCb(const double& p)
{
  assert(do_mag_soft_bias_estimation_);

  const auto nd = p * 1e-5 / tbs::kGeomagScale;  // nT/s/√Hz -> /s/√Hz
  return eskf_.setMagSoftBiasProcNoiseDensity(nd);
}

bool ErrorStateKalmanFilterNode::gravProcNoiseDensityCb(const double& p)
{
  assert(do_grav_estimation_);

  const auto nd = p * 1e-6 * tbs::kGravity;  // ug/s/√Hz -> m/s^3/√Hz
  return eskf_.setGravProcNoiseDensity(nd);
}

void ErrorStateKalmanFilterNode::imuRawCb(const ImuMsg::ConstSharedPtr& imu_raw)
{
  // Compute IMU time
  const auto cur_time = ros2::chronoFromRosTime(imu_raw->header.stamp);

  // Initialization
  if (!imu_raw_) {
    if (!eskf_.initialize(
          Vector3d::Zero(),                                                     // Init position
          Vector3d::Constant(math::sqr(kInitPosStddev)).asDiagonal(),           // Init position cov
          Vector3d::Zero(),                                                     // Init velocity
          Vector3d::Constant(math::sqr(kInitVelStddev)).asDiagonal(),           // Init velocity cov
          Quaterniond::Identity(),                                              // Init quaternion
          Vector3d::Constant(math::sqr(kInitRotStddev)).asDiagonal(),           // Init rotation cov
          Vector3d::Zero(),                                                     // Init accel bias
          Vector3d::Constant(math::sqr(initAccelBiasStddev())).asDiagonal(),    // Init accel bias cov
          Vector3d::Zero(),                                                     // Init gyro bias
          Vector3d::Constant(math::sqr(initGyroBiasStddev())).asDiagonal(),     // Init gyro bias cov
          Vector3d::Zero(),                                                     // Init mag hard bias
          Vector3d::Constant(math::sqr(initMagHardBiasStddev())).asDiagonal(),  // Init mag hard bias cov
          Matrix3d::Identity(),                                                 // Init mag soft bias
          Vector6d::Constant(math::sqr(initMagSoftBiasStddev())).asDiagonal(),  // Init mag soft bias cov
          tbs::kGravity,                                                        // Init gravity
          math::sqr(initGravBiasStddev()),                                      // Init gravity var
          cur_time)) {
      TOBAS_ERROR("Failed to initialize ESKF.");
      return;
    }

    imu_raw_ = imu_raw;
    return;
  }

  // Update IMU message
  imu_raw_ = imu_raw;

  // Measure IMU
  const auto& acc_meas = imu_raw->accel.data;
  const auto& gyro_meas = imu_raw->gyro.data;
  const auto grav_cov = adaptive_grav_noise_ ? calcGravMeasNoiseCov(acc_meas) : fixed_grav_cov_;
  eskf_.measureIMU(acc_meas, gyro_meas, fixed_acc_cov_, fixed_gyro_cov_, grav_cov, cur_time);

  // Do not publish any messages if filtered IMU message is not ready
  if (!imu_filt_) {
    return;
  }

  // Create odometry message
  auto odom = std::make_unique<OdomMsg>();
  fillOdometryMsg(*odom);

  // Create TF message
  tf_.header.stamp = odom->header.stamp;
  kdl::transformKDLToMsg(odom->frame, tf_.transform);

  // Publish odometry and TF
  odom_pub_->publish(std::move(odom));
  tf_br_.sendTransform(tf_);

  // Publish feedback
  publishFeedback(imu_raw->header);
}

void ErrorStateKalmanFilterNode::imuFiltCb(const ImuMsg::ConstSharedPtr& imu_filt)
{
  imu_filt_ = imu_filt;
}

void ErrorStateKalmanFilterNode::magCb(const MagMsg::ConstSharedPtr& mag)
{
  if (!imu_raw_) {
    return;
  }

  mag_ = mag;

  // 最初の地磁気を受け取った時にGPSが受け取れていなければ，ひとまず最初の地磁気ベクトルを参照とする．
  if (!mag_ref_set_) {
    setMagneticFieldRef(mag_->mag.data);
    return;
  }

  const auto& mag_meas = mag->mag.data;
  const auto stamp = ros2::chronoFromRosTime(mag->header.stamp);

  // バイアス推定を行う場合は3軸，行わない場合はヨーのみ更新
  if (do_mag_hard_bias_estimation_ || do_mag_soft_bias_estimation_) {
    eskf_.measureMagneticField3d(mag_meas, fixed_mag_cov_, stamp);
  }
  else {
    eskf_.measureMagneticFieldHead(mag_meas, fixed_head_var_, stamp);
  }
}

void ErrorStateKalmanFilterNode::baroCb(const BaroMsg::ConstSharedPtr& baro)
{
  if (!imu_raw_) {
    return;
  }

  // 気圧高度の初期値
  // TODO: IMUフレームに変換
  if (!baro_) {
    alt_0_ = tbs::pressureToAltitude(baro->pressure);
  }

  baro_ = baro;

  // TODO: baro_offsetを考慮
  const auto z_abs = tbs::pressureToAltitude(baro->pressure);
  const auto z_var = fixed_baro_alt_var_;
  const auto z_m = z_abs - alt_0_;
  const auto stamp = ros2::chronoFromRosTime(baro->header.stamp);
  eskf_.measureAltitude(z_m, z_var, stamp);
}

void ErrorStateKalmanFilterNode::gnssCb(const GnssMsg::ConstSharedPtr& gnss)
{
  if (!imu_raw_ || !imu_filt_) {
    return;
  }

  gnss_fix_ = (gnss->fix_type == tobas_msgs::msg::Gnss::FIX_3D);
  if (!gnss_fix_) {
    return;
  }

  const auto& vel_meas = gnss->ground_speed.data;
  const auto& pos_cov = adaptive_gnss_noise_ ? gnss->position_covariance : fixed_gnss_pos_cov_;
  const auto& vel_cov = adaptive_gnss_noise_ ? gnss->velocity_covariance : fixed_gnss_vel_cov_;

  if (!gnss_) {
    // GNSSの初期位置
    // TODO: IMUフレームに変換
    lat_0_ = gnss->latitude;
    lon_0_ = gnss->longitude;
    alt_0_ = gnss->altitude;

    // GNSSの初期位置を発行
    publishGnssOrigin(lat_0_, lon_0_, alt_0_);

    // GNSSの初期値から地磁気の参照値を求める
    // TODO: 位置の変化に合わせてオンラインで参照値を求める
    const auto mag = geomag::elementsFromGeodetic(lat_0_, lon_0_, alt_0_, tim::yearFraction());
    const Vector3d mag_W(mag.east, mag.north, -mag.down);  // ENU coordinates
    if (!setMagneticFieldRef(mag_W)) {
      return;
    }

    // 初めてGNSSを受け取った位置速度で初期化 (でないと姿勢に過大なフィードバックが入ってしまう)
    // FIXME: 既に他の位置情報が入っている場合は初期化すべきでない
    if (!eskf_.initializePosition(Vector3d::Zero(), pos_cov)) {
      TOBAS_ERROR("Failed to initialize position.");
      return;
    }
    if (!eskf_.initializeVelocity(vel_meas, vel_cov)) {
      TOBAS_ERROR("Failed to initialize velocity.");
      return;
    }
  }

  gnss_ = gnss;

  // 位置の観測値
  tbs::gnssToCartRelative(gnss->latitude, gnss->longitude, lat_0_, lon_0_, pos_meas_.x(), pos_meas_.y());
  pos_meas_.z() = gnss->altitude - alt_0_;  // FIXME: BaroとGNSSが両方有効のときに高度情報が競合する

  // 共分散
  gnss_cov_.topLeftCorner<3, 3>() = pos_cov;
  gnss_cov_.bottomRightCorner<3, 3>() = vel_cov;

  // ESKFを更新
  const Vector3d imu2gnss = gnss_offset_ - imu_offset_;
  const auto& gyro_meas = imu_filt_->gyro.data;
  const auto stamp = ros2::chronoFromRosTime(gnss->header.stamp);
  gnss_anomaly_score_ = eskf_.measurePosVel(pos_meas_, vel_meas, gnss_cov_, imu2gnss, gyro_meas, stamp);
}

void ErrorStateKalmanFilterNode::poseCb(const PoseMsg::ConstSharedPtr& msg)
{
  if (!imu_raw_ || !imu_filt_) {
    return;
  }

  const auto& pose = msg->frame.frame;

  Quaterniond quat;
  pose.M.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());

  const auto stamp = ros2::chronoFromRosTime(msg->header.stamp);
  eskf_.measurePose(pose.p.data, quat, msg->frame.covariance, Vector3d::Zero(), stamp);  // TODO: オフセット指定
}

void ErrorStateKalmanFilterNode::getGnssOriginCb(
  const GetOrigin::Request::ConstSharedPtr&,
  const GetOrigin::Response::SharedPtr& res)
{
  if (!gnss_fix_) {
    res->success = false;
    res->message = "GNSS position is not fixed.";
    return;
  }

  res->latitude = lat_0_;
  res->longitude = lon_0_;

  res->success = true;
  res->message.clear();
}

void ErrorStateKalmanFilterNode::setGnssOriginCb(
  const SetOrigin::Request::ConstSharedPtr& req,
  const SetOrigin::Response::SharedPtr& res)
{
  if (!gnss_fix_) {
    res->success = false;
    res->message = "GNSS position is not fixed.";
    return;
  }

  lat_0_ = req->latitude;
  lon_0_ = req->longitude;

  publishGnssOrigin(lat_0_, lon_0_, alt_0_);

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(ErrorStateKalmanFilterNode)
