#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/fluid_pressure.hpp>
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

#include <tobas_msgs_adapter/Imu.hpp>
#include <tobas_msgs_adapter/MagneticField.hpp>
#include <tobas_msgs_adapter/Gps.hpp>
#include <tobas_msgs_adapter/Odometry.hpp>
#include <tobas_msgs/srv/get_gnss_origin.hpp>
#include <tobas_msgs/srv/set_gnss_origin.hpp>
#include <tobas_debug_msgs_adapter/ObserverFeedback.hpp>

#include "../include/state_estimation_eskf/eskf.hpp"

using namespace std;
using namespace Eigen;

class ObserverNode : public tobas::BaseNode
{
  using self = ObserverNode;
  using super = tobas::BaseNode;

  using ImuMsg = tobas_msgs::Imu;
  using MagMsg = tobas_msgs::MagneticField;
  using BarMsg = sensor_msgs::msg::FluidPressure;
  using GpsMsg = tobas_msgs::Gps;
  using OdomMsg = tobas_msgs::Odometry;
  using FeedbackMsg = tobas_debug_msgs::ObserverFeedback;

  using GetGnssOrigin = tobas_msgs::srv::GetGnssOrigin;
  using SetGnssOrigin = tobas_msgs::srv::SetGnssOrigin;

public:
  explicit ObserverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
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

  Vector3d pos_meas_;
  Matrix3d grav_cov_ = Matrix3d::Zero();
  double yaw_var_;
  double acc_bias_noise_var_;   // 加速度バイアスののプロセスノイズの分散
  double gyro_bias_noise_var_;  // ジャイロバイアスののプロセスノイズの分散
  double grav_noise_var_;       // 重力加速度のプロセスノイズの分散

  eskf::ErrorStateKalmanFilter eskf_;

  // Static parameters
  string frame_id_;
  bool use_bar_;
  bool use_gps_;
  bool do_acc_bias_estimation_;
  bool do_gyro_bias_estimation_;
  bool do_grav_estimation_;
  Vector3d imu_offset_;  // [m] ルートリンクに対するIMUの位置 (Local)
  Vector3d bar_offset_;  // [m] ルートリンクに対する気圧センサの位置 (Local)
  Vector3d gps_offset_;  // [m] ルートリンクに対するGPSレシーバの位置 (Local)

  // Publishers
  ros2::PublisherPtr<OdomMsg> odom_pub_;
  ros2::PublisherPtr<FeedbackMsg> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<ImuMsg> imu_sub_;
  ros2::SubscriberPtr<ImuMsg> imu_filtered_sub_;
  ros2::SubscriberPtr<MagMsg> mag_sub_;
  ros2::SubscriberPtr<BarMsg> bar_sub_;
  ros2::SubscriberPtr<GpsMsg> gps_sub_;

  // Services
  ros2::ServicePtr<GetGnssOrigin> get_gnss_origin_ss_;
  ros2::ServicePtr<SetGnssOrigin> set_gnss_origin_ss_;

  // TF
  geometry_msgs::msg::TransformStamped tf_;
  tf2_ros::TransformBroadcaster tf_br_;

  void getStaticRosParams();
  void fillOdometryMsg(OdomMsg& odom) const;

  bool gravityVarianceCb(const long& p);
  bool yawVarianceCb(const long& p);
  bool accBiasNoiseVarianceLog10Cb(const long& p);
  bool gyroBiasNoiseVarianceLog10Cb(const long& p);
  bool gravityNoiseVarianceLog10Cb(const long& p);

  void imuCb(const ImuMsg::ConstSharedPtr& imu);
  void imuFilteredCb(const ImuMsg::ConstSharedPtr& imu_filtered);
  void magCb(const MagMsg::ConstSharedPtr& mag);
  void barCb(const BarMsg::ConstSharedPtr& bar);
  void gpsCb(const GpsMsg::ConstSharedPtr& gps);

  void
  getGnssOriginCb(const GetGnssOrigin::Request::ConstSharedPtr& req, const GetGnssOrigin::Response::SharedPtr& res);
  void
  setGnssOriginCb(const SetGnssOrigin::Request::ConstSharedPtr& req, const SetGnssOrigin::Response::SharedPtr& res);
};

ObserverNode::ObserverNode(const rclcpp::NodeOptions& options) : super(tobas::kObserverNode, options), tf_br_(this)
{
  getStaticRosParams();

  // Initialize ESKF
  const double init_acc_bias_stddev = do_acc_bias_estimation_ ? eskf::kInitAccBiasStddev : 0;
  const double init_gyro_bias_stddev = do_gyro_bias_estimation_ ? eskf::kInitGyroBiasStddev : 0;
  const double init_grav_stddev = do_grav_estimation_ ? eskf::kInitGravStddev : 0;
  eskf_.initialize(
    Vector3d::Zero(),                                                   // Init position
    Vector3d::Zero(),                                                   // Init velocity
    Quaterniond::Identity(),                                            // Init quaternion
    Vector3d::Constant(math::sqr(eskf::kInitPosStddev)).asDiagonal(),   // Init position cov
    Vector3d::Constant(math::sqr(eskf::kInitVelStddev)).asDiagonal(),   // Init velocity cov
    Vector3d::Constant(math::sqr(eskf::kInitRotStddev)).asDiagonal(),   // Init rotation cov
    Vector3d::Constant(math::sqr(init_acc_bias_stddev)).asDiagonal(),   // Init accel bias cov
    Vector3d::Constant(math::sqr(init_gyro_bias_stddev)).asDiagonal(),  // Init gyro bias cov
    math::sqr(init_grav_stddev)                                         // Init gravity var
  );

  // Fill the static part of the transform message
  tf_.header.frame_id = tobas::kWorldFrame;
  tf_.child_frame_id = frame_id_;

  // Register dynamic parameters
  addDynamicIntParam("gravity_variance", &self::gravityVarianceCb, this, 500, 1, 1000);
  addDynamicIntParam("yaw_variance", &self::yawVarianceCb, this, 1, 1, 100);
  addDynamicIntParam("acc_bias_noise_var_log10", &self::accBiasNoiseVarianceLog10Cb, this, -5, -12, 0);
  addDynamicIntParam("gyro_bias_noise_var_log10", &self::gyroBiasNoiseVarianceLog10Cb, this, -9, -12, 0);
  addDynamicIntParam("gravity_noise_var_log10", &self::gravityNoiseVarianceLog10Cb, this, -7, -12, 0);
  publishDynamicParameterDescriptions();

  // Register publishers
  odom_pub_ = createPublisher<OdomMsg>(tobas::kOdometryTopic);
  feedback_pub_ = createPublisher<FeedbackMsg>(tobas::kObserverFeedbackTopic);

  // Register subscribers
  imu_sub_ = createSubscriber(tobas::kImuTopic, &self::imuCb, this);
  imu_filtered_sub_ = createSubscriber(tobas::kImuLpfTopic, &self::imuFilteredCb, this);
  mag_sub_ = createSubscriber(tobas::kMagTopic, &self::magCb, this);
  if (use_bar_)
    bar_sub_ = createSubscriber(tobas::kAirPressureTopic, &self::barCb, this);
  if (use_gps_)
    gps_sub_ = createSubscriber(tobas::kGpsTopic, &self::gpsCb, this);

  // Register service servers
  get_gnss_origin_ss_ = createService<GetGnssOrigin>(tobas::kGetGnssOriginSrv, &self::getGnssOriginCb, this);
  set_gnss_origin_ss_ = createService<SetGnssOrigin>(tobas::kSetGnssOriginSrv, &self::setGnssOriginCb, this);
}

void ObserverNode::getStaticRosParams()
{
  frame_id_ = getStringParam("frame_id", "");
  use_bar_ = getBoolParam("use_barometer", eskf::kDefaultUseBarometer);
  use_gps_ = getBoolParam("use_gps", eskf::kDefaultUseGps);
  do_acc_bias_estimation_ = getBoolParam("do_acc_bias_estimation", eskf::kDefaultDoAccBiasEstimation);
  do_gyro_bias_estimation_ = getBoolParam("do_gyro_bias_estimation", eskf::kDefaultDoGyroBiasEstimation);
  do_grav_estimation_ = getBoolParam("do_gravity_estimation", eskf::kDefaultDoGravEstimation);

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

void ObserverNode::fillOdometryMsg(OdomMsg& odom) const
{
  const Vector3d W_Pos_WI = eskf_.getPosition();
  const Vector3d W_Vel_WI = eskf_.getVelocity();
  const Quaterniond W_Rot_B = eskf_.getQuaternion();
  const Quaterniond B_Rot_W = W_Rot_B.conjugate();
  const Vector3d B_grav = B_Rot_W * Vector3d(0, 0, -tobas_std::kGravity);
  const Vector3d B_Acc = imu_filtered_->accel.data - eskf_.getAccelBias() + B_grav;  // 重力を除いた加速度
  const Vector3d B_Gyro = imu_filtered_->gyro.data - eskf_.getGyroBias();

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
  odom.orientation_covariance = eskf_.getOrientationCovariance();

  // Angular velocity (Local)
  odom.twist.rot.data = B_Gyro;
  odom.gyro_covariance = imu_->gyro_covariance;

  // Linear acceleration (Local)
  odom.accel.linear.data = B_Acc;
  odom.accel_covariance = imu_->accel_covariance;

  // Angular acceleration (Local)
  odom.accel.angular.fill(nan(tobas::kUnknown));
  odom.dgyro_covariance.fill(nan(tobas::kUnknown));
}

bool ObserverNode::gravityVarianceCb(const long& p)
{
  grav_cov_.diagonal().fill(p);
  return true;
}

bool ObserverNode::yawVarianceCb(const long& p)
{
  yaw_var_ = p;
  return true;
}

bool ObserverNode::accBiasNoiseVarianceLog10Cb(const long& p)
{
  if (!do_acc_bias_estimation_)
  {
    acc_bias_noise_var_ = 0.;
    TOBAS_INFO("Change of accel bias noise variance is ignored because accel bias estimation is disabled.");
    return false;
  }

  acc_bias_noise_var_ = exp10(p);
  return true;
}

bool ObserverNode::gyroBiasNoiseVarianceLog10Cb(const long& p)
{
  if (!do_gyro_bias_estimation_)
  {
    gyro_bias_noise_var_ = 0.;
    TOBAS_INFO("Change of gyro bias noise variance is ignored because gyro bias estimation is disabled.");
    return false;
  }

  gyro_bias_noise_var_ = exp10(p);
  return true;
}

bool ObserverNode::gravityNoiseVarianceLog10Cb(const long& p)
{
  if (!do_grav_estimation_)
  {
    grav_noise_var_ = 0.;
    TOBAS_INFO("Change of gravity noise variance is ignored because gravity estimation is disabled.");
    return false;
  }

  grav_noise_var_ = exp10(p);
  return true;
}

void ObserverNode::imuCb(const ImuMsg::ConstSharedPtr& imu)
{
  if (imu_ == nullptr)
  {
    imu_ = imu;
    return;
  }

  // Compute delta time
  const auto dt = (imu->header.stamp - imu_->header.stamp).seconds();
  imu_ = imu;

  // Check IMU time gap
  if (dt == 0)
  {
    TOBAS_ERROR("The time gap between 2 IMU messages is 0.");
    return;
  }
  if (dt < 0)
  {
    TOBAS_ERROR("The time gap between 2 IMU messages is negative: ", dt, " [s]");
    return;
  }
  if (dt > eskf::kImuTimeGapThreshold)
  {
    TOBAS_WARN("The time gap between 2 IMU messages is too large: ", dt, " [s]");
  }

  // 観測ノイズの分散を計算
  const auto acc_noise_var = imu->accel_covariance.diagonal().mean();
  const auto gyro_noise_var = imu->gyro_covariance.diagonal().mean();

  // 事前予測
  eskf_.predictIMU(
    imu->accel.data, imu->gyro.data, acc_noise_var, gyro_noise_var, acc_bias_noise_var_, gyro_bias_noise_var_,
    grav_noise_var_, dt);

  // 重力方向の観測
  eskf_.measureGravity(imu->accel.data, grav_cov_);

  // フィルタリング済みIMUを受け取るまでは発行しない
  if (imu_filtered_ == nullptr)
  {
    TOBAS_INFO_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kImuLpfTopic, "\".");
    return;
  }

  // 推定状態を発行
  auto odom = std::make_unique<OdomMsg>();
  fillOdometryMsg(*odom);
  odom_pub_->publish(move(odom));

  // TFを発行
  tf_.header.stamp = odom->header.stamp;
  transformKDLToMsg(odom->frame, tf_.transform);
  tf_br_.sendTransform(tf_);

  // フィードバックを発行
  auto feedback = std::make_unique<FeedbackMsg>();
  feedback->header = imu->header;
  feedback->acc_bias.data = eskf_.getAccelBias();
  feedback->gyro_bias.data = eskf_.getGyroBias();
  feedback->gravity = eskf_.getGravity();
  feedback->acc_bias_covariance = eskf_.getAccelBiasCovariance();
  feedback->gyro_bias_covariance = eskf_.getGyroBiasCovariance();
  feedback->gravity_variance = eskf_.getGravityVariance();
  feedback->gps_anormaly_score = gps_anormaly_score_;
  feedback_pub_->publish(move(feedback));
}

void ObserverNode::imuFilteredCb(const ImuMsg::ConstSharedPtr& imu_filtered)
{
  imu_filtered_ = imu_filtered;
}

void ObserverNode::magCb(const MagMsg::ConstSharedPtr& mag)
{
  if (imu_ == nullptr)
    return;

  // 最初の地磁気を受け取った時にGPSが受け取れていなければ，ひとまず最初のヨー角をゼロ点とする．
  if (mag_ == nullptr && gps_ == nullptr)
    yaw_0_ = atan2(mag->magnetic_field.y(), mag->magnetic_field.x());

  mag_ = mag;

  const auto yaw_meas = algo::wrapPi(yaw_0_ - atan2(mag->magnetic_field.y(), mag->magnetic_field.x()));
  eskf_.measureYaw(yaw_meas, yaw_var_);
}

void ObserverNode::barCb(const BarMsg::ConstSharedPtr& bar)
{
  if (imu_ == nullptr)
    return;

  // 気圧高度の初期値
  // TODO: IMUフレームに変換
  if (bar_ == nullptr)
    alt_0_bar_ = tobas_std::pressureToAltitude(bar->fluid_pressure);

  bar_ = bar;

  double z_abs, z_var;
  tobas_std::pressureToAltitude(bar->fluid_pressure, bar->variance, z_abs, z_var);

  // TODO: bar_offsetを考慮
  const auto z_m = z_abs - alt_0_bar_;
  eskf_.measureAltitude(z_m, z_var);
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

    // GPSの初期値から地磁気の参照値を求める
    // TODO: 位置の変化に合わせてオンラインで参照値を求める
    const auto mag = geomag::elementsFromGeodetic(lat_0_, lon_0_, alt_0_gps_, tobas_std::yearFraction());
    yaw_0_ = atan2(-mag.east, mag.north);

    // 初めてGNSSを受け取った位置で初期化 (でないと姿勢に過大なフィードバックが入ってしまう)
    // FIXME: 既に他の位置情報が入っている場合は初期化すべきでない
    eskf_.setPosition(Vector3d::Zero());
  }

  gps_ = gps;

  // TODO: 遅延を考慮
  // const auto delay = imu_->header.stamp - gps->header.stamp;
  // cout << "GNSS delay: " << delay << endl;

  // 位置の観測値
  tobas_std::gpsToCartRelative(gps->latitude, gps->longitude, lat_0_, lon_0_, pos_meas_.x(), pos_meas_.y());
  pos_meas_.z() = gps->altitude - alt_0_gps_;  // FIXME: 気圧高度と競合しそう

  // ESKFを更新
  const Vector3d imu2gps = gps_offset_ - imu_offset_;
  gps_anormaly_score_ = eskf_.measurePosVel(
    pos_meas_, gps->position_covariance, gps->ground_speed.data, gps->velocity_covariance, imu2gps, imu_->gyro.data);

  // 異常度が高すぎる場合は警告
  if (gps_anormaly_score_ > eskf::kAnormalyScoreThreshold)
    TOBAS_WARN_THROTTLE(eskf::kWarnPeriod, "The position estimation using GNSS is unstable.");
}

void ObserverNode::getGnssOriginCb(
  const GetGnssOrigin::Request::ConstSharedPtr&,
  const GetGnssOrigin::Response::SharedPtr& res)
{
  if (!gps_fix_)
  {
    res->success = false;
    res->message = "GNSS is not fixed.";
    return;
  }

  res->latitude = lat_0_;
  res->longitude = lon_0_;

  res->success = true;
  return;
}

void ObserverNode::setGnssOriginCb(
  const SetGnssOrigin::Request::ConstSharedPtr& req,
  const SetGnssOrigin::Response::SharedPtr& res)
{
  if (!gps_fix_)
  {
    res->success = false;
    res->message = "GNSS is not fixed.";
    return;
  }

  lat_0_ = req->latitude;
  lon_0_ = req->longitude;

  res->success = true;
  return;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ObserverNode)
