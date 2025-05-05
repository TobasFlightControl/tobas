#include <tf2_ros/transform_broadcaster.h>

#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_std_tools/time.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_geomag/core.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tobas_msgs/msg/geodetic_coordinates.hpp>
#include <tobas_msgs/msg/fluid_pressure_with_variance_stamped.hpp>
#include <tobas_msgs_adapter/imu_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/magnetic_field_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/gnss.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs/srv/get_gnss_origin.hpp>
#include <tobas_msgs/srv/set_gnss_origin.hpp>
#include <tobas_debug_msgs_adapter/observer_feedback.hpp>

#include "../include/tobas_eskf/eskf.hpp"

using namespace std;
using namespace Eigen;

class ObserverNode : public tobas::BaseNode
{
  using self = ObserverNode;
  using super = tobas::BaseNode;

  using ImuMsg = tobas_msgs::ImuWithCovarianceStamped;
  using MagMsg = tobas_msgs::MagneticFieldWithCovarianceStamped;
  using BarMsg = tobas_msgs::msg::FluidPressureWithVarianceStamped;
  using GnssMsg = tobas_msgs::Gnss;
  using OdomMsg = tobas_msgs::Odometry;
  using GnssOriginMsg = tobas_msgs::msg::GeodeticCoordinates;
  using FeedbackMsg = tobas_debug_msgs::ObserverFeedback;

  using GetOrigin = tobas_msgs::srv::GetGnssOrigin;
  using SetOrigin = tobas_msgs::srv::SetGnssOrigin;

  // Default parameters
  static constexpr bool kDefaultUseBarometer = false;
  static constexpr bool kDefaultUseGnss = true;
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

  // その他
  static constexpr double kAnomalyScoreThreshold = 10.;  // [-]

public:
  explicit ObserverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // 固定値
  double lat_0_;       // 緯度のゼロ点 (Base Frame)
  double lon_0_;       // 経度のゼロ点 (Base Frame)
  double alt_0_gnss_;  // GNSS高度のゼロ点 (Base Frame)
  double alt_0_bar_;   // 気圧高度のゼロ点 (Base Frame)

  Vector3d pos_meas_;
  kdl::Vector dgyro_;
  ImuMsg::ConstSharedPtr imu_;
  MagMsg::ConstSharedPtr mag_;
  BarMsg::ConstSharedPtr bar_;
  GnssMsg::ConstSharedPtr gnss_;
  bool mag_ref_set_ = false;  // 地磁気の参照値が設定されているかどうか
  bool gnss_fix_ = false;
  double gnss_anomaly_score_ = 0.;

  eskf::ErrorStateKalmanFilter eskf_;

  // Static parameters
  string frame_id_;
  bool use_bar_;
  bool use_gnss_;
  bool do_acc_bias_estimation_;
  bool do_gyro_bias_estimation_;
  bool do_mag_hard_bias_estimation_;
  bool do_mag_soft_bias_estimation_;
  bool do_grav_estimation_;
  Vector3d imu_offset_;   // [m] ルートリンクに対するIMUの位置 (Local)
  Vector3d bar_offset_;   // [m] ルートリンクに対する気圧センサの位置 (Local)
  Vector3d gnss_offset_;  // [m] ルートリンクに対するGNSSレシーバの位置 (Local)

  // Dynamic parameters
  double grav_meas_var_intercept_;  // [m^2/s^4] 重力方向の加速度の観測の不確かさの最小値
  double grav_meas_var_slope_;  // [m/s^2] 重力方向の観測の，加速度ノルム誤差に対する比例定数

  // Publishers
  ros2::PublisherPtr<OdomMsg> odom_pub_;
  ros2::PublisherPtr<GnssOriginMsg> gnss_origin_pub_;
  ros2::PublisherPtr<FeedbackMsg> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<ImuMsg> imu_sub_;
  ros2::SubscriberPtr<MagMsg> mag_sub_;
  ros2::SubscriberPtr<BarMsg> bar_sub_;
  ros2::SubscriberPtr<GnssMsg> gnss_sub_;

  // Services
  ros2::ServiceServerPtr<GetOrigin> get_gnss_origin_ss_;
  ros2::ServiceServerPtr<SetOrigin> set_gnss_origin_ss_;

  // TF
  geometry_msgs::msg::TransformStamped tf_;
  tf2_ros::TransformBroadcaster tf_br_;

  void getStaticRosParams();
  bool setMagneticFieldRef(const Vector3d& mag_W);
  void fillOdometryMsg(OdomMsg& odom) const;
  void publishGNSSOrigin();
  void publishFeedback(const std_msgs::msg::Header& header);
  double computeGravMeasVariance(const Vector3d& acc) const;

  double initAccelBiasStddev() const;
  double initGyroBiasStddev() const;
  double initMagHardBiasStddev() const;
  double initMagSoftBiasStddev() const;
  double initGravBiasStddev() const;

  bool accBiasProcNoiseDensityCb(const long& nd_ug);
  bool gyroBiasProcNoiseDensityCb(const long& nd_udps);
  bool magHardBiasProcNoiseDensityCb(const long& nd_u);
  bool magSoftBiasProcNoiseDensityCb(const long& nd_u);
  bool gravProcNoiseDensityCb(const long& ud_ug);
  bool gravMeasVarInterceptCb(const long& p);
  bool gravMeasVarSlopeCb(const long& p);

  void imuCb(const ImuMsg::ConstSharedPtr& imu);
  void magCb(const MagMsg::ConstSharedPtr& mag);
  void barCb(const BarMsg::ConstSharedPtr& bar);
  void gnssCb(const GnssMsg::ConstSharedPtr& gnss);

  void getGnssOriginCb(const GetOrigin::Request::ConstSharedPtr& req, const GetOrigin::Response::SharedPtr& res);
  void setGnssOriginCb(const SetOrigin::Request::ConstSharedPtr& req, const SetOrigin::Response::SharedPtr& res);
};

ObserverNode::ObserverNode(const rclcpp::NodeOptions& options) : super(tobas::node::kObserver, options), tf_br_(this)
{
  getStaticRosParams();

  // Fill the static part of the transform message
  tf_.header.frame_id = tobas::kWorldFrame;
  tf_.child_frame_id = frame_id_;

  // Register dynamic parameters
  if (do_acc_bias_estimation_) {
    addDynamicIntParam("acc_bias_proc_noise_density", &self::accBiasProcNoiseDensityCb, this, 20, 0, 1000);  // [ug/√Hz]
  }
  if (do_gyro_bias_estimation_) {
    addDynamicIntParam(
      "gyro_bias_proc_noise_density", &self::gyroBiasProcNoiseDensityCb, this, 1000, 0, 10000);  // [udps/√Hz]
  }
  if (do_mag_hard_bias_estimation_) {
    addDynamicIntParam(
      "mag_hard_bias_proc_noise_density", &self::magHardBiasProcNoiseDensityCb, this, 1000, 0, 10000);  // [u/√Hz]
  }
  if (do_mag_soft_bias_estimation_) {
    addDynamicIntParam(
      "mag_soft_bias_proc_noise_density", &self::magSoftBiasProcNoiseDensityCb, this, 1000, 0, 10000);  // [u/√Hz]
  }
  if (do_grav_estimation_) {
    addDynamicIntParam("grav_noise_proc_noise_density", &self::gravProcNoiseDensityCb, this, 20, 0, 1000);  // [ug/√Hz]
  }

  addDynamicIntParam("grav_meas_var_intercept", &self::gravMeasVarInterceptCb, this, 1, 1, 100);
  addDynamicIntParam("grav_meas_var_slope", &self::gravMeasVarSlopeCb, this, 100, 0, 1000);

  // Register publishers
  odom_pub_ = createPublisher<OdomMsg>(tobas::kOdometryTopic);
  gnss_origin_pub_ = createPublisher<GnssOriginMsg>(tobas::kGnssOriginTopic, true, true);
  feedback_pub_ = createPublisher<FeedbackMsg>(tobas::kObsvFeedbackTopic);

  // Register subscribers
  imu_sub_ = createSubscriber(tobas::kImuTopic, &self::imuCb, this);
  mag_sub_ = createSubscriber(tobas::kMagTopic, &self::magCb, this);
  if (use_bar_) {
    bar_sub_ = createSubscriber(tobas::kAirPressureTopic, &self::barCb, this);
  }
  if (use_gnss_) {
    gnss_sub_ = createSubscriber(tobas::kGnssTopic, &self::gnssCb, this);
  }

  // Register service servers
  get_gnss_origin_ss_ = createService<GetOrigin>(tobas::kGetGnssOriginSrv, &self::getGnssOriginCb, this);
  set_gnss_origin_ss_ = createService<SetOrigin>(tobas::kSetGnssOriginSrv, &self::setGnssOriginCb, this);
}

void ObserverNode::getStaticRosParams()
{
  frame_id_ = getStringParam("frame_id", "unknown");  // 空文字だとTFが警告文を出す
  use_bar_ = getBoolParam("use_barometer", kDefaultUseBarometer);
  use_gnss_ = getBoolParam("use_gnss", kDefaultUseGnss);
  do_acc_bias_estimation_ = getBoolParam("do_acc_bias_estimation", kDefaultDoAccBiasEstimation);
  do_gyro_bias_estimation_ = getBoolParam("do_gyro_bias_estimation", kDefaultDoGyroBiasEstimation);
  do_mag_hard_bias_estimation_ = getBoolParam("do_mag_hard_bias_estimation", kDefaultDoMagHardBiasEstimation);
  do_mag_soft_bias_estimation_ = getBoolParam("do_mag_soft_bias_estimation", kDefaultDoMagSoftBiasEstimation);
  do_grav_estimation_ = getBoolParam("do_gravity_estimation", kDefaultDoGravEstimation);

  const auto imu_offset = getDoubleArrayParam("imu_offset", vector<double>(3, 0.));
  const auto bar_offset = getDoubleArrayParam("barometer_offset", vector<double>(3, 0.));
  const auto gnss_offset = getDoubleArrayParam("gnss_offset", vector<double>(3, 0.));
  imu_offset_ = Map<const Vector3d>(imu_offset.data());
  bar_offset_ = Map<const Vector3d>(bar_offset.data());
  gnss_offset_ = Map<const Vector3d>(gnss_offset.data());
}

bool ObserverNode::setMagneticFieldRef(const Vector3d& mag_W)
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
    tobas_std::eulerFromQuaternion(R_W_B.x(), R_W_B.y(), R_W_B.z(), R_W_B.w(), old_roll, old_pitch, old_yaw);

    // 地磁気をヨー角のみ機体と一致し，XY軸が地面と平行な地上座標系Gに移す．
    const AngleAxisd R_W_G(old_yaw, Vector3d::UnitZ());
    const auto mag_G = R_W_G.inverse() * (R_W_B * mag_->mag.mag.data);  // 後ろから計算することで計算量を削減
    const auto mx = mag_G.x();
    const auto my = mag_G.y();

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

  TOBAS_INFO("Reference magnetic field is set to be: ", mag_W.transpose());
  mag_ref_set_ = true;

  return true;
}

void ObserverNode::fillOdometryMsg(OdomMsg& odom) const
{
  const Vector3d W_Pos_WI = eskf_.getPosition();
  const Vector3d W_Vel_WI = eskf_.getVelocity();
  const Quaterniond W_Rot_B = eskf_.getQuaternion();
  const Quaterniond B_Rot_W = W_Rot_B.conjugate();
  const Vector3d B_grav = B_Rot_W * Vector3d(0, 0, -eskf_.getGravity());
  const Vector3d B_Acc = imu_->imu.imu.accel.data - eskf_.getAccelBias() + B_grav;  // 重力を除いた加速度
  const Vector3d B_Gyro = imu_->imu.imu.gyro.data - eskf_.getGyroBias();

  // Header
  odom.header.stamp = imu_->header.stamp;
  odom.header.frame_id = tobas::kWorldFrame;

  // Status
  if (!gnss_fix_) {
    odom.status = tobas_msgs::msg::Odometry::POSITION_LOST;
  }
  else {
    odom.status = tobas_msgs::msg::Odometry::NO_ERROR;
  }

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
  odom.gyro_covariance = imu_->imu.gyro_covariance;

  // Linear acceleration (Local)
  odom.accel.linear.data = B_Acc;
  odom.accel_covariance = imu_->imu.accel_covariance;

  // Angular acceleration (Local)
  odom.accel.angular = dgyro_;
  odom.dgyro_covariance.fill(NAN);
}

void ObserverNode::publishGNSSOrigin()
{
  auto gnss_origin = std::make_unique<GnssOriginMsg>();

  gnss_origin->header.stamp = get_clock()->now();
  gnss_origin->latitude = lat_0_;
  gnss_origin->longitude = lon_0_;
  gnss_origin->altitude = alt_0_gnss_;

  gnss_origin_pub_->publish(move(gnss_origin));
}

void ObserverNode::publishFeedback(const std_msgs::msg::Header& header)
{
  auto feedback = std::make_unique<FeedbackMsg>();

  feedback->header = header;

  feedback->position = eskf_.getPosition();
  feedback->velocity = eskf_.getVelocity();
  feedback->hamilton = eskf_.getHamilton();
  feedback->accel_bias = eskf_.getAccelBias();
  feedback->gyro_bias = eskf_.getGyroBias();
  feedback->mag_hard_bias = eskf_.getMagHardBias();
  feedback->mag_soft_bias = eskf_.getMagSoftBias();
  feedback->gravity = eskf_.getGravity();

  feedback->position_cov = eskf_.getPositionCovariance();
  feedback->velocity_cov = eskf_.getVelocityCovariance();
  feedback->rotation_cov = eskf_.getRotationCovariance();
  feedback->accel_bias_cov = eskf_.getAccelBiasCovariance();
  feedback->gyro_bias_cov = eskf_.getGyroBiasCovariance();
  feedback->mag_hard_bias_cov = eskf_.getMagHardBiasCovariance();
  feedback->mag_soft_bias_cov = eskf_.getMagSoftBiasCovariance();
  feedback->gravity_var = eskf_.getGravityVariance();

  feedback->gnss_anomaly_score = gnss_anomaly_score_;

  feedback_pub_->publish(move(feedback));
}

double ObserverNode::computeGravMeasVariance(const Vector3d& acc) const
{
  // 加速度のL2ノルムから重力方向の観測の不確かさを決める．
  // 加速度の大きさと重力加速度との誤差が大きいほど重力以外の加速度が生じているため加速度による姿勢の観測が不確かだと考えるのは直感的だが，
  // その誤差は正規分布に従うわけではなく一様に確かでもないため，誤差をそのまま標準偏差とすることには何の根拠もない．
  // 実際，重力方向の分散を下げると，並進移動時に進行方向への加速度により実際よりも大きく傾いていると判断され，
  // 制御器が姿勢を戻そうとし，並進方向の加速度の追従が遅れ，位置制御が振動するという因果関係がある．
  // 動的加速度が陽にモデルに含まれていない以上，その不確かさの決定はヒューリスティックにならざるを得ない．
  // 実用的には動作時の追従遅れと静止時の収束速度のトレードオフを考慮して決定するしかないだろう．
  const auto acc_norm_diff = fabs(acc.norm() - eskf_.getGravity());  // TODO: モデルから推定した動的加速度を考慮
  return grav_meas_var_intercept_ + grav_meas_var_slope_ * acc_norm_diff;  // TODO: 1次関数以外のプロファイルを検討
}

double ObserverNode::initAccelBiasStddev() const
{
  return do_acc_bias_estimation_ ? 1. : 0.;
}

double ObserverNode::initGyroBiasStddev() const
{
  return do_gyro_bias_estimation_ ? 0.1 : 0.;
}

double ObserverNode::initMagHardBiasStddev() const
{
  return do_mag_hard_bias_estimation_ ? 0.1 : 0.;
}

double ObserverNode::initMagSoftBiasStddev() const
{
  return do_mag_soft_bias_estimation_ ? 0.1 : 0.;
}

double ObserverNode::initGravBiasStddev() const
{
  return do_grav_estimation_ ? 0.1 : 0.;
}

bool ObserverNode::accBiasProcNoiseDensityCb(const long& nd_ug)
{
  assert(do_acc_bias_estimation_);

  const auto nd = nd_ug * 1e-6 * tobas_std::kGravity;  // [m/s^2/√Hz]
  return eskf_.setAccBiasProcNoiseDensity(nd);
}

bool ObserverNode::gyroBiasProcNoiseDensityCb(const long& nd_udps)
{
  assert(do_gyro_bias_estimation_);

  const auto nd = nd_udps * 1e-6 * tobas_std::kDeg2Rad;  // [rad/s/√Hz]
  return eskf_.setGyroBiasProcNoiseDensity(nd);
}

bool ObserverNode::magHardBiasProcNoiseDensityCb(const long& nd_u)
{
  assert(do_mag_hard_bias_estimation_);

  const auto nd = nd_u * 1e-6;  // [/√Hz]
  return eskf_.setMagHardBiasProcNoiseDensity(nd);
}

bool ObserverNode::magSoftBiasProcNoiseDensityCb(const long& nd_u)
{
  assert(do_mag_soft_bias_estimation_);

  const auto nd = nd_u * 1e-6;  // [/√Hz]
  return eskf_.setMagSoftBiasProcNoiseDensity(nd);
}

bool ObserverNode::gravProcNoiseDensityCb(const long& nd_ug)
{
  assert(do_grav_estimation_);

  const auto nd = nd_ug * 1e-6 * tobas_std::kGravity;  // [m/s^2/√Hz]
  return eskf_.setGravProcNoiseDensity(nd);
}

bool ObserverNode::gravMeasVarInterceptCb(const long& p)
{
  grav_meas_var_intercept_ = p;
  return true;
}

bool ObserverNode::gravMeasVarSlopeCb(const long& p)
{
  grav_meas_var_slope_ = p;
  return true;
}

void ObserverNode::imuCb(const ImuMsg::ConstSharedPtr& imu)
{
  // Compute IMU time
  const auto cur_time = ros2::chronoFromRosTime(imu->header.stamp);

  // Initialization
  if (!imu_) {
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
          tobas_std::kGravity,                                                  // Init gravity
          math::sqr(initGravBiasStddev()),                                      // Init gravity var
          cur_time)) {
      TOBAS_ERROR("Failed to initialize ESKF.");
      return;
    }

    imu_ = imu;
    return;
  }

  // Update D-Gyro (No filtering)
  const auto dt = (imu->header.stamp - imu_->header.stamp).seconds();
  dgyro_ = (imu->imu.imu.gyro - imu_->imu.imu.gyro) / dt;

  // Update IMU message
  imu_ = imu;

  // Measure IMU
  const auto grav_meas_noise_var = computeGravMeasVariance(imu->imu.imu.accel.data);
  const auto grav_cov = Vector3d::Constant(grav_meas_noise_var).asDiagonal();
  eskf_.measureIMU(
    imu->imu.imu.accel.data,
    imu->imu.imu.gyro.data,
    imu->imu.accel_covariance,
    imu->imu.gyro_covariance,
    grav_cov,
    cur_time);

  // Create odometry message
  auto odom = std::make_unique<OdomMsg>();
  fillOdometryMsg(*odom);

  // Create TF message
  tf_.header.stamp = odom->header.stamp;
  transformKDLToMsg(odom->frame, tf_.transform);

  // Publish odometry and TF
  odom_pub_->publish(move(odom));
  tf_br_.sendTransform(tf_);

  // Publish feedback
  publishFeedback(imu->header);
}

void ObserverNode::magCb(const MagMsg::ConstSharedPtr& mag)
{
  if (!imu_) {
    return;
  }

  mag_ = mag;

  // 最初の地磁気を受け取った時にGPSが受け取れていなければ，ひとまず最初の地磁気ベクトルを参照とする．
  if (!mag_ref_set_) {
    setMagneticFieldRef(mag_->mag.mag.data);
    return;
  }

  // バイアス推定を行う場合は3軸，行わない場合はヨーのみ更新
  if (do_mag_hard_bias_estimation_ || do_mag_soft_bias_estimation_) {
    eskf_.measureMagneticField3d(mag->mag.mag.data, mag->mag.covariance, ros2::chronoFromRosTime(mag->header.stamp));
  }
  else {
    eskf_.measureMagneticFieldYaw(mag->mag.mag.data, mag->mag.covariance, ros2::chronoFromRosTime(mag->header.stamp));
  }
}

void ObserverNode::barCb(const BarMsg::ConstSharedPtr& bar)
{
  if (!imu_) {
    return;
  }

  // 気圧高度の初期値
  // TODO: IMUフレームに変換
  if (!bar_) {
    alt_0_bar_ = tobas_std::pressureToAltitude(bar->pressure.pressure);
  }

  bar_ = bar;

  double z_abs, z_var;
  tobas_std::pressureToAltitude(bar->pressure.pressure, bar->pressure.variance, z_abs, z_var);

  // TODO: bar_offsetを考慮
  const auto z_m = z_abs - alt_0_bar_;
  eskf_.measureAltitude(z_m, z_var, ros2::chronoFromRosTime(bar->header.stamp));
}

void ObserverNode::gnssCb(const GnssMsg::ConstSharedPtr& gnss)
{
  if (!imu_) {
    return;
  }

  gnss_fix_ = (gnss->fix_type == tobas_msgs::msg::Gnss::FIX_3D);
  if (!gnss_fix_) {
    return;
  }

  if (!gnss_) {
    // GNSSの初期位置
    // TODO: IMUフレームに変換
    lat_0_ = gnss->latitude;
    lon_0_ = gnss->longitude;
    alt_0_gnss_ = gnss->altitude;

    // GNSSの初期位置を発行
    publishGNSSOrigin();

    // GNSSの初期値から地磁気の参照値を求める
    // TODO: 位置の変化に合わせてオンラインで参照値を求める
    const auto mag = geomag::elementsFromGeodetic(lat_0_, lon_0_, alt_0_gnss_, tobas_std::yearFraction());
    Vector3d mag_W(mag.north, -mag.east, -mag.down);  // NWU coordinates
    if (!setMagneticFieldRef(mag_W)) {
      return;
    }

    // 初めてGNSSを受け取った位置速度で初期化 (でないと姿勢に過大なフィードバックが入ってしまう)
    // FIXME: 既に他の位置情報が入っている場合は初期化すべきでない
    if (!eskf_.initializePosition(Vector3d::Zero(), gnss->position_covariance)) {
      TOBAS_ERROR("Failed to initialize position.");
      return;
    }
    if (!eskf_.initializeVelocity(gnss->ground_speed.data, gnss->velocity_covariance)) {
      TOBAS_ERROR("Failed to initialize velocity.");
      return;
    }
  }

  gnss_ = gnss;

  // 位置の観測値
  tobas_std::gnssToCartRelative(gnss->latitude, gnss->longitude, lat_0_, lon_0_, pos_meas_.x(), pos_meas_.y());
  pos_meas_.z() = gnss->altitude - alt_0_gnss_;  // FIXME: 気圧高度と競合しそう

  // ESKFを更新
  const Vector3d imu2gnss = gnss_offset_ - imu_offset_;
  gnss_anomaly_score_ = eskf_.measurePosVel(
    pos_meas_,
    gnss->position_covariance,
    gnss->ground_speed.data,
    gnss->velocity_covariance,
    imu2gnss,
    imu_->imu.imu.gyro.data,
    ros2::chronoFromRosTime(gnss->header.stamp));

  // 異常度が高すぎる場合は警告
  if (gnss_anomaly_score_ > kAnomalyScoreThreshold) {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "The position estimation using GNSS is unstable.");
  }
}

void ObserverNode::getGnssOriginCb(const GetOrigin::Request::ConstSharedPtr&, const GetOrigin::Response::SharedPtr& res)
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
  return;
}

void ObserverNode::setGnssOriginCb(
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

  publishGNSSOrigin();

  res->success = true;
  res->message.clear();
  return;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ObserverNode)
