#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

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

#include <tobas_msgs/msg/geodetic_coordinates.hpp>
#include <tobas_msgs/msg/fluid_pressure_with_variance_stamped.hpp>
#include <tobas_msgs_adapter/imu_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/magnetic_field_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/gps.hpp>
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
  using GpsMsg = tobas_msgs::Gps;
  using OdomMsg = tobas_msgs::Odometry;
  using GpsOriginMsg = tobas_msgs::msg::GeodeticCoordinates;
  using FeedbackMsg = tobas_debug_msgs::ObserverFeedback;

  using GetOrigin = tobas_msgs::srv::GetGnssOrigin;
  using SetOrigin = tobas_msgs::srv::SetGnssOrigin;

  // Default parameters
  static constexpr bool kDefaultUseBarometer = false;
  static constexpr bool kDefaultUseGps = true;
  static constexpr bool kDefaultDoAccBiasEstimation = false;
  static constexpr bool kDefaultDoGyroBiasEstimation = true;
  static constexpr bool kDefaultDoMagHardBiasEstimation = true;
  static constexpr bool kDefaultDoMagSoftBiasEstimation = false;
  static constexpr bool kDefaultDoGravEstimation = true;

  // 標準偏差の初期値
  // 共分散行列は成長は遅いが収束は割と速いから，大きすぎるくらいで適当に決めてよい
  static constexpr double kInitPosStddev = 5.;      // [m]
  static constexpr double kInitVelStddev = 1.;      // [m/s]
  static constexpr double kInitRotStddev = M_PI_4;  // [rad]
  static constexpr double kInitMagStddev = 0.5;     // [-]

  // その他
  static constexpr double kAnormalyScoreThreshold = 10.;  // [-]

public:
  explicit ObserverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // 固定値
  double lat_0_;      // 緯度のゼロ点 (Base Frame)
  double lon_0_;      // 経度のゼロ点 (Base Frame)
  double alt_0_gps_;  // GPS高度のゼロ点 (Base Frame)
  double alt_0_bar_;  // 気圧高度のゼロ点 (Base Frame)

  Vector3d pos_meas_;
  kdl::Vector dgyro_;
  ImuMsg::ConstSharedPtr imu_;
  MagMsg::ConstSharedPtr mag_;
  BarMsg::ConstSharedPtr bar_;
  GpsMsg::ConstSharedPtr gps_;
  bool gps_fix_ = false;
  double gps_anormaly_score_ = 0.;

  eskf::ErrorStateKalmanFilter eskf_;

  // Static parameters
  string frame_id_;
  bool use_bar_;
  bool use_gps_;
  bool do_acc_bias_estimation_;
  bool do_gyro_bias_estimation_;
  bool do_mag_hard_bias_estimation_;
  bool do_mag_soft_bias_estimation_;
  bool do_grav_estimation_;
  Vector3d imu_offset_;  // [m] ルートリンクに対するIMUの位置 (Local)
  Vector3d bar_offset_;  // [m] ルートリンクに対する気圧センサの位置 (Local)
  Vector3d gps_offset_;  // [m] ルートリンクに対するGPSレシーバの位置 (Local)

  // Dynamic parameters
  double grav_meas_var_intercept_;  // [m^2/s^4] 重力方向の加速度の観測の不確かさの最小値
  double grav_meas_var_slope_;  // [m/s^2] 重力方向の観測の，加速度ノルム誤差に対する比例定数

  // Publishers
  ros2::PublisherPtr<OdomMsg> odom_pub_;
  ros2::PublisherPtr<GpsOriginMsg> gps_origin_pub_;
  ros2::PublisherPtr<FeedbackMsg> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<ImuMsg> imu_sub_;
  ros2::SubscriberPtr<MagMsg> mag_sub_;
  ros2::SubscriberPtr<BarMsg> bar_sub_;
  ros2::SubscriberPtr<GpsMsg> gps_sub_;

  // Services
  ros2::ServiceServerPtr<GetOrigin> get_gnss_origin_ss_;
  ros2::ServiceServerPtr<SetOrigin> set_gnss_origin_ss_;

  // TF
  geometry_msgs::msg::TransformStamped tf_;
  tf2_ros::TransformBroadcaster tf_br_;

  void getStaticRosParams();
  void setMagneticFieldRefAndInitializeBias(const Vector3d& mag_ref);
  void fillOdometryMsg(OdomMsg& odom) const;
  void publishGPSOrigin();
  void publishFeedback(const std_msgs::msg::Header& header);
  double computeGravMeasVariance(const Vector3d& acc) const;

  double initAccelBiasStddev() const;
  double initGyroBiasStddev() const;
  double initMagHardBiasStddev() const;
  double initMagSoftBiasStddev() const;
  double initGravBiasStddev() const;

  bool gravMeasVarInterceptCb(const long& p);
  bool gravMeasVarSlopeCb(const long& p);
  bool accBiasProcNoiseVarLog10Cb(const long& p);
  bool gyroBiasProcNoiseVarLog10Cb(const long& p);
  bool magHardBiasProcNoiseVarLog10Cb(const long& p);
  bool magSoftBiasProcNoiseVarLog10Cb(const long& p);
  bool gravProcNoiseVarLog10Cb(const long& p);

  void imuCb(const ImuMsg::ConstSharedPtr& imu);
  void magCb(const MagMsg::ConstSharedPtr& mag);
  void barCb(const BarMsg::ConstSharedPtr& bar);
  void gpsCb(const GpsMsg::ConstSharedPtr& gps);

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
  if (do_acc_bias_estimation_)
    addDynamicIntParam("acc_bias_proc_noise_var_log10", &self::accBiasProcNoiseVarLog10Cb, this, -5, -12, 0);
  if (do_gyro_bias_estimation_)
    addDynamicIntParam("gyro_bias_proc_noise_var_log10", &self::gyroBiasProcNoiseVarLog10Cb, this, -9, -12, 0);
  if (do_mag_hard_bias_estimation_)
    addDynamicIntParam("mag_hard_bias_proc_noise_var_log10", &self::magHardBiasProcNoiseVarLog10Cb, this, -5, -12, 0);
  if (do_mag_soft_bias_estimation_)
    addDynamicIntParam("mag_soft_bias_proc_noise_var_log10", &self::magSoftBiasProcNoiseVarLog10Cb, this, -5, -12, 0);
  if (do_grav_estimation_)
    addDynamicIntParam("grav_noise_proc_var_log10", &self::gravProcNoiseVarLog10Cb, this, -7, -12, 0);
  addDynamicIntParam("grav_meas_var_intercept", &self::gravMeasVarInterceptCb, this, 1, 1, 100);
  addDynamicIntParam("grav_meas_var_slope", &self::gravMeasVarSlopeCb, this, 100, 0, 1000);

  // Register publishers
  odom_pub_ = createPublisher<OdomMsg>(tobas::kOdometryTopic);
  gps_origin_pub_ = createPublisher<GpsOriginMsg>(tobas::kGpsOriginTopic, true, true);
  feedback_pub_ = createPublisher<FeedbackMsg>(tobas::kObsvFeedbackTopic);

  // Register subscribers
  imu_sub_ = createSubscriber(tobas::kImuTopic, &self::imuCb, this);
  mag_sub_ = createSubscriber(tobas::kMagTopic, &self::magCb, this);
  if (use_bar_)
    bar_sub_ = createSubscriber(tobas::kAirPressureTopic, &self::barCb, this);
  if (use_gps_)
    gps_sub_ = createSubscriber(tobas::kGNSSTopic, &self::gpsCb, this);

  // Register service servers
  get_gnss_origin_ss_ = createService<GetOrigin>(tobas::kGetGnssOriginSrv, &self::getGnssOriginCb, this);
  set_gnss_origin_ss_ = createService<SetOrigin>(tobas::kSetGnssOriginSrv, &self::setGnssOriginCb, this);
}

void ObserverNode::getStaticRosParams()
{
  frame_id_ = getStringParam("frame_id", "unknown");  // 空文字だとTFが警告文を出す
  use_bar_ = getBoolParam("use_barometer", kDefaultUseBarometer);
  use_gps_ = getBoolParam("use_gps", kDefaultUseGps);
  do_acc_bias_estimation_ = getBoolParam("do_acc_bias_estimation", kDefaultDoAccBiasEstimation);
  do_gyro_bias_estimation_ = getBoolParam("do_gyro_bias_estimation", kDefaultDoGyroBiasEstimation);
  do_mag_hard_bias_estimation_ = getBoolParam("do_mag_hard_bias_estimation", kDefaultDoMagHardBiasEstimation);
  do_mag_soft_bias_estimation_ = getBoolParam("do_mag_soft_bias_estimation", kDefaultDoMagSoftBiasEstimation);
  do_grav_estimation_ = getBoolParam("do_gravity_estimation", kDefaultDoGravEstimation);

  const auto imu_offset = getDoubleArrayParam("imu_offset", vector<double>(3, 0.));
  const auto bar_offset = getDoubleArrayParam("barometer_offset", vector<double>(3, 0.));
  const auto gps_offset = getDoubleArrayParam("gps_offset", vector<double>(3, 0.));
  imu_offset_ = Map<const Vector3d>(imu_offset.data());
  bar_offset_ = Map<const Vector3d>(bar_offset.data());
  gps_offset_ = Map<const Vector3d>(gps_offset.data());

  // 加速度バイアスのZ成分と重力加速度の分離は困難だと思われるため，どちらか一方のみを許容
  if (do_acc_bias_estimation_ && do_grav_estimation_)
    TOBAS_EXIT("You cannot enable both accelerometer bias estimation and gravity estimation.");
}

void ObserverNode::setMagneticFieldRefAndInitializeBias(const Vector3d& mag_ref)
{
  eskf_.setMagneticFieldRef(mag_ref);

  // 地磁気の参照値が変わったらバイアスを初期化する
  eskf_.initializeMagHardBias(Vector3d::Zero(), Vector3d::Constant(math::sqr(initMagHardBiasStddev())).asDiagonal());
  eskf_.initializeMagSoftBias(
    Matrix3d::Identity(), Vector6d::Constant(math::sqr(initMagSoftBiasStddev())).asDiagonal());
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
  if (!gps_fix_)
    odom.status = tobas_msgs::msg::Odometry::POSITION_LOST;
  else
    odom.status = tobas_msgs::msg::Odometry::NO_ERROR;

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

void ObserverNode::publishGPSOrigin()
{
  auto gps_origin = std::make_unique<GpsOriginMsg>();

  gps_origin->header.stamp = get_clock()->now();
  gps_origin->latitude = lat_0_;
  gps_origin->longitude = lon_0_;
  gps_origin->altitude = alt_0_gps_;

  gps_origin_pub_->publish(move(gps_origin));
}

void ObserverNode::publishFeedback(const std_msgs::msg::Header& header)
{
  auto feedback = std::make_unique<FeedbackMsg>();

  feedback->header = header;

  feedback->position = eskf_.getPosition();
  feedback->velocity = eskf_.getVelocity();
  feedback->hamilton = eskf_.getHamilton();
  feedback->magnetic_field = eskf_.getMagneticField();
  feedback->accel_bias = eskf_.getAccelBias();
  feedback->gyro_bias = eskf_.getGyroBias();
  feedback->mag_hard_bias = eskf_.getMagHardBias();
  feedback->mag_soft_bias = eskf_.getMagSoftBias();
  feedback->gravity = eskf_.getGravity();

  feedback->position_cov = eskf_.getPositionCovariance();
  feedback->velocity_cov = eskf_.getVelocityCovariance();
  feedback->rotation_cov = eskf_.getRotationCovariance();
  feedback->magnetic_field_cov = eskf_.getMagneticFieldCovariance();
  feedback->accel_bias_cov = eskf_.getAccelBiasCovariance();
  feedback->gyro_bias_cov = eskf_.getGyroBiasCovariance();
  feedback->mag_hard_bias_cov = eskf_.getMagHardBiasCovariance();
  feedback->mag_soft_bias_cov = eskf_.getMagSoftBiasCovariance();
  feedback->gravity_var = eskf_.getGravityVariance();

  feedback->gps_anormaly_score = gps_anormaly_score_;

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

bool ObserverNode::accBiasProcNoiseVarLog10Cb(const long& p)
{
  assert(do_acc_bias_estimation_);
  return eskf_.setAccBiasProcNoiseVar(exp10(p));
}

bool ObserverNode::gyroBiasProcNoiseVarLog10Cb(const long& p)
{
  assert(do_gyro_bias_estimation_);
  return eskf_.setGyroBiasProcNoiseVar(exp10(p));
}

bool ObserverNode::magHardBiasProcNoiseVarLog10Cb(const long& p)
{
  assert(do_mag_hard_bias_estimation_);
  return eskf_.setMagHardBiasProcNoiseVar(exp10(p));
}

bool ObserverNode::magSoftBiasProcNoiseVarLog10Cb(const long& p)
{
  assert(do_mag_soft_bias_estimation_);
  return eskf_.setMagSoftBiasProcNoiseVar(exp10(p));
}

bool ObserverNode::gravProcNoiseVarLog10Cb(const long& p)
{
  assert(do_grav_estimation_);
  return eskf_.setGravProcNoiseVar(exp10(p));
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
  if (imu_ == nullptr)
  {
    if (!eskf_.initialize(
          Vector3d::Zero(),                                                     // Init position
          Vector3d::Constant(math::sqr(kInitPosStddev)).asDiagonal(),           // Init position cov
          Vector3d::Zero(),                                                     // Init velocity
          Vector3d::Constant(math::sqr(kInitVelStddev)).asDiagonal(),           // Init velocity cov
          Quaterniond::Identity(),                                              // Init quaternion
          Vector3d::Constant(math::sqr(kInitRotStddev)).asDiagonal(),           // Init rotation cov
          Vector3d::UnitX(),                                                    // Init magnetic field
          Vector3d::Constant(math::sqr(kInitMagStddev)).asDiagonal(),           // Init magnetic field cov
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
          cur_time))
    {
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
    imu->imu.imu.accel.data, imu->imu.imu.gyro.data, imu->imu.accel_covariance, imu->imu.gyro_covariance, grav_cov,
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
  if (imu_ == nullptr)
    return;

  if (mag_ == nullptr)
  {
    // 最初の地磁気データで初期化
    if (!eskf_.initializeMagneticField(mag->mag.mag.data, Vector3d::Constant(math::sqr(kInitMagStddev)).asDiagonal()))
    {
      TOBAS_ERROR("Failed to initialize magnetic field.");
      return;
    }

    // 最初の地磁気を受け取った時にGPSが受け取れていなければ，ひとまず最初の地磁気ベクトルを参照値とする．
    // これをしないと，ヨーの初期誤差によってロール，ピッチの推定が不安定になることがある．
    if (gps_ == nullptr)
      setMagneticFieldRefAndInitializeBias(mag->mag.mag.data);
  }

  mag_ = mag;

  eskf_.measureMagneticField(mag->mag.mag.data, mag->mag.covariance, ros2::chronoFromRosTime(mag->header.stamp));
}

void ObserverNode::barCb(const BarMsg::ConstSharedPtr& bar)
{
  if (imu_ == nullptr)
    return;

  // 気圧高度の初期値
  // TODO: IMUフレームに変換
  if (bar_ == nullptr)
    alt_0_bar_ = tobas_std::pressureToAltitude(bar->pressure.pressure);

  bar_ = bar;

  double z_abs, z_var;
  tobas_std::pressureToAltitude(bar->pressure.pressure, bar->pressure.variance, z_abs, z_var);

  // TODO: bar_offsetを考慮
  const auto z_m = z_abs - alt_0_bar_;
  eskf_.measureAltitude(z_m, z_var, ros2::chronoFromRosTime(bar->header.stamp));
}

void ObserverNode::gpsCb(const GpsMsg::ConstSharedPtr& gps)
{
  if (imu_ == nullptr)
    return;

  gps_fix_ = (gps->fix_type == tobas_msgs::msg::Gps::FIX_3D);
  if (!gps_fix_)
    return;

  if (gps_ == nullptr)
  {
    // GPSの初期位置
    // TODO: IMUフレームに変換
    lat_0_ = gps->latitude;
    lon_0_ = gps->longitude;
    alt_0_gps_ = gps->altitude;

    // GPSの初期位置を発行
    publishGPSOrigin();

    // GPSの初期値から地磁気の参照値を求める
    // TODO: 位置の変化に合わせてオンラインで参照値を求める
    const auto mag = geomag::elementsFromGeodetic(lat_0_, lon_0_, alt_0_gps_, tobas_std::yearFraction());
    Vector3d mag_ref(mag.north, -mag.east, -mag.down);  // NWU coordinates
    setMagneticFieldRefAndInitializeBias(mag_ref);

    // 初めてGNSSを受け取った位置で初期化 (でないと姿勢に過大なフィードバックが入ってしまう)
    // FIXME: 既に他の位置情報が入っている場合は初期化すべきでない
    if (!eskf_.initializePosition(Vector3d::Zero(), gps->position_covariance))
    {
      TOBAS_ERROR("Failed to initialize position.");
      return;
    }
  }

  gps_ = gps;

  // 位置の観測値
  tobas_std::gpsToCartRelative(gps->latitude, gps->longitude, lat_0_, lon_0_, pos_meas_.x(), pos_meas_.y());
  pos_meas_.z() = gps->altitude - alt_0_gps_;  // FIXME: 気圧高度と競合しそう

  // ESKFを更新
  const Vector3d imu2gps = gps_offset_ - imu_offset_;
  gps_anormaly_score_ = eskf_.measurePosVel(
    pos_meas_, gps->position_covariance, gps->ground_speed.data, gps->velocity_covariance, imu2gps,
    imu_->imu.imu.gyro.data, ros2::chronoFromRosTime(gps->header.stamp));

  // 異常度が高すぎる場合は警告
  if (gps_anormaly_score_ > kAnormalyScoreThreshold)
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "The position estimation using GNSS is unstable.");
}

void ObserverNode::getGnssOriginCb(const GetOrigin::Request::ConstSharedPtr&, const GetOrigin::Response::SharedPtr& res)
{
  if (!gps_fix_)
  {
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
  if (!gps_fix_)
  {
    res->success = false;
    res->message = "GNSS position is not fixed.";
    return;
  }

  lat_0_ = req->latitude;
  lon_0_ = req->longitude;

  publishGPSOrigin();

  res->success = true;
  res->message.clear();
  return;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ObserverNode)
