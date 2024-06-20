#include <tobas_std_tools/algorithm.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/boost.hpp>
#include <tobas_std_tools/exception.hpp>
#include <tobas_std_tools/debug.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_eigen_tools/iostream.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_kdl_msgs/conversion/kdl_msg.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_msgs/conversions/msg_msg.hpp>

#include "../include/state_estimation_eskf/eskf_ros.hpp"

using namespace std;
using namespace Eigen;

namespace et = eigen_tools;

namespace state_estimation_eskf
{
ErrorStateKalmanFilterRos::ErrorStateKalmanFilterRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name), server_(pnh_)
{
  PRINT_DEBUG("ErrorStateKalmanFilterRos::ErrorStateKalmanFilterRos");

  getRosParams();
  drone_.loadFromParam(nh_);

  // Initialize ESKF
  const double init_acc_bias_stddev = do_acc_bias_estimation_ ? kInitAccBiasStddev : 0;
  const double init_gyro_bias_stddev = do_gyro_bias_estimation_ ? kInitGyroBiasStddev : 0;
  const double init_grav_stddev = do_grav_estimation_ ? kInitGravStddev : 0;
  eskf_.initialize(
    Vector3d::Zero(),                                                   // Init position
    Vector3d::Zero(),                                                   // Init velocity
    Quaterniond::Identity(),                                            // Init quaternion
    Vector3d::Constant(math::sqr(kInitPosStddev)).asDiagonal(),         // Init position cov
    Vector3d::Constant(math::sqr(kInitVelStddev)).asDiagonal(),         // Init velocity cov
    Vector3d::Constant(math::sqr(kInitRotStddev)).asDiagonal(),         // Init rotation cov
    Vector3d::Constant(math::sqr(init_acc_bias_stddev)).asDiagonal(),   // Init accel bias cov
    Vector3d::Constant(math::sqr(init_gyro_bias_stddev)).asDiagonal(),  // Init gyro bias cov
    math::sqr(init_grav_stddev)                                         // Init gravity var
  );

  // Fill the static part of the transform message
  tf_.header.frame_id = tobas::kWorldFrame;
  tf_.child_frame_id = drone_.tree().getRootName();

  // Register publishers
  odom_pub_ = nh_.advertise<OdomMsg>(tobas::kOdometryTopic, 1);
  feedback_pub_ = nh_.advertise<FeedbackMsg>("eskf_feedback", 1);

  // Register subscribers
  imu_sub_ = nh_.subscribe(tobas::kImuTopic, 1, &self::imuCb, this, tcpNoDelay());
  imu_filtered_sub_ = nh_.subscribe(tobas::kImuLpfTopic, 1, &self::imuFilteredCb, this, tcpNoDelay());
  mag_sub_ = nh_.subscribe(tobas::kMagTopic, 1, &self::magCb, this, tcpNoDelay());
  if (use_bar_)
    bar_sub_ = nh_.subscribe(tobas::kAirPressureTopic, 1, &self::barCb, this, tcpNoDelay());
  if (use_gps_)
    gps_sub_ = nh_.subscribe(tobas::kGpsTopic, 1, &self::gpsCb, this, tcpNoDelay());

  // Register service servers
  get_gnss_origin_ss_ = nh_.advertiseService(tobas::kGetGnssOriginSrv, &self::getGnssOriginCb, this);
  set_gnss_origin_ss_ = nh_.advertiseService(tobas::kSetGnssOriginSrv, &self::setGnssOriginCb, this);

  // Dynamic Reconfigureの設定．この時点で1度コールバックが呼ばれる．
  server_.setCallback(boost::bind(&self::dynamicReconfigureCb, this, _1, _2));
}

void ErrorStateKalmanFilterRos::getRosParams()
{
  tobas_ros::getParam(pnh_, "use_barometer", use_bar_, kDefaultUseBarometer);
  tobas_ros::getParam(pnh_, "use_gps", use_gps_, kDefaultUseGps);
  tobas_ros::getParam(pnh_, "do_acc_bias_estimation", do_acc_bias_estimation_, kDefaultDoAccBiasEstimation);
  tobas_ros::getParam(pnh_, "do_gyro_bias_estimation", do_gyro_bias_estimation_, kDefaultDoGyroBiasEstimation);
  tobas_ros::getParam(pnh_, "do_gravity_estimation", do_grav_estimation_, kDefaultDoGravEstimation);
  tobas_ros::getParam(pnh_, "imu_offset", imu_offset_, Vector3d::Zero());
  tobas_ros::getParam(pnh_, "barometer_offset", bar_offset_, Vector3d::Zero());
  tobas_ros::getParam(pnh_, "gps_offset", gps_offset_, Vector3d::Zero());

  // 加速度バイアスのZ成分と重力加速度の分離は困難だと思われるため，どちらか一方のみを許容
  if (do_acc_bias_estimation_ && do_grav_estimation_)
    TOBAS_EXIT("You cannot enable both accelerometer bias estimation and gravity estimation.");
}

ErrorStateKalmanFilterRos::OdomMsg::ConstPtr ErrorStateKalmanFilterRos::makeOdometryMsg() const
{
  const Vector3d W_Pos_WI = eskf_.getPosition();
  const Vector3d W_Vel_WI = eskf_.getVelocity();
  const Quaterniond W_Rot_B = eskf_.getQuaternion();
  const Quaterniond B_Rot_W = W_Rot_B.conjugate();
  const Vector3d B_grav = B_Rot_W * Vector3d(0, 0, -tobas::kGravity);
  const Vector3d B_Acc = imu_filtered_->accel.data - eskf_.getAccelBias() + B_grav;  // 重力を除いた加速度
  const Vector3d B_Gyro = imu_filtered_->gyro.data - eskf_.getGyroBias();

  const auto odom = boost::make_shared<OdomMsg>();

  // Header
  odom->header.stamp = imu_->header.stamp;
  odom->header.frame_id = tobas::kWorldFrame;

  // Status
  if (!gps_fix_)
    odom->status = OdomMsg::POSITION_LOST;
  else
    odom->status = OdomMsg::NO_ERROR;

  // Position (Global): IMU frame -> Base frame
  odom->frame.p.data = W_Pos_WI - W_Rot_B * imu_offset_;
  odom->position_covariance = eskf_.getPositionCovariance();

  // Linear velocity (Local): IMU frame -> Base frame
  odom->twist.vel.data = B_Rot_W * W_Vel_WI - B_Gyro.cross(imu_offset_);
  odom->linear_velocity_covariance = B_Rot_W * eskf_.getVelocityCovariance() * W_Rot_B;

  // Orientation (Global)
  odom->frame.M.data = W_Rot_B.toRotationMatrix();
  odom->orientation_covariance = eskf_.getOrientationCovariance();

  // Angular velocity (Local)
  odom->twist.rot.data = B_Gyro;
  odom->angular_velocity_covariance = imu_->gyro_covariance;

  // Linear acceleration (Local)
  odom->accel.linear.data = B_Acc;
  odom->linear_acceleration_covariance = imu_->accel_covariance;

  // Angular acceleration (Local)
  odom->accel.angular.fill(nan(tobas::kUnknown));
  odom->angular_acceleration_covariance.fill(nan(tobas::kUnknown));

  return odom;
}

void ErrorStateKalmanFilterRos::imuCb(const ImuMsg::ConstPtr& imu)
{
  if (imu_ == nullptr)
  {
    imu_ = imu;
    return;
  }

  // Compute delta time
  const auto dt = (imu->header.stamp - imu_->header.stamp).toSec();
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
  if (dt > kImuTimeGapThreshold)
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
    TOBAS_INFO_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kImuLpfTopic);
    return;
  }

  // 推定状態を発行
  const auto odom = makeOdometryMsg();
  odom_pub_.publish(odom);

  // TFを発行
  tf_.header.stamp = odom->header.stamp;
  transformKDLToMsg(odom->frame, tf_.transform);
  tf_br_.sendTransform(tf_);

  // フィードバックを発行
  const auto feedback = boost::make_shared<FeedbackMsg>();
  feedback->header = imu->header;
  feedback->acc_bias.data = eskf_.getAccelBias();
  feedback->gyro_bias.data = eskf_.getGyroBias();
  feedback->gravity = eskf_.getGravity();
  feedback->acc_bias_covariance = eskf_.getAccelBiasCovariance();
  feedback->gyro_bias_covariance = eskf_.getGyroBiasCovariance();
  feedback->gravity_variance = eskf_.getGravityVariance();
  feedback->gps_anormaly_score = gps_anormaly_score_;
  feedback_pub_.publish(feedback);
}

void ErrorStateKalmanFilterRos::imuFilteredCb(const ImuMsg::ConstPtr& imu_filtered)
{
  imu_filtered_ = imu_filtered;
}

void ErrorStateKalmanFilterRos::magCb(const MagMsg::ConstPtr& mag)
{
  if (imu_ == nullptr)
    return;

  // 最初の地磁気を受け取った時にGPSが受け取れていなければ，ひとまず最初のヨー角をゼロ点とする．
  if (mag_ == nullptr && gps_ == nullptr)
    yaw_0_ = atan2(mag->magnetic_field.y(), mag->magnetic_field.x());

  mag_ = mag;

  const double yaw_meas = tobas_std::wrapPi(yaw_0_ - atan2(mag->magnetic_field.y(), mag->magnetic_field.x()));
  eskf_.measureYaw(yaw_meas, yaw_var_);
}

void ErrorStateKalmanFilterRos::barCb(const BarMsg::ConstPtr& bar)
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
  const double z_m = z_abs - alt_0_bar_;
  eskf_.measureAltitude(z_m, z_var);
}

void ErrorStateKalmanFilterRos::gpsCb(const GpsMsg::ConstPtr& gps)
{
  if (imu_ == nullptr)
    return;

  gps_fix_ = (gps->fix_type == GpsMsg::FIX_3D);
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
    const auto mag = tobas::geomag(lat_0_, lon_0_, alt_0_gps_);
    yaw_0_ = atan2(-mag.east, mag.north);

    // 初めてGNSSを受け取った位置で初期化 (でないと姿勢に過大なフィードバックが入ってしまう)
    // FIXME: 既に他の位置情報が入っている場合は初期化すべきでない
    eskf_.setPosition(Vector3d::Zero());
  }

  gps_ = gps;

  // 位置の観測値
  tobas_std::gpsToCartRelative(gps->latitude, gps->longitude, lat_0_, lon_0_, pos_meas_.x(), pos_meas_.y());
  pos_meas_.z() = gps->altitude - alt_0_gps_;  // FIXME: 気圧高度と競合しそう

  // ESKFを更新
  const Vector3d imu2gps = gps_offset_ - imu_offset_;
  gps_anormaly_score_ = eskf_.measurePosVel(
    pos_meas_, gps->position_covariance, gps->ground_speed.data, gps->velocity_covariance, imu2gps, imu_->gyro.data);

  // 異常度が高すぎる場合は警告
  if (gps_anormaly_score_ > kAnormalyScoreThreshold)
    TOBAS_WARN_THROTTLE(kWarnPeriod, "The position estimation using GNSS is unstable.");
}

bool ErrorStateKalmanFilterRos::getGnssOriginCb(
  tobas_msgs::GetGnssOriginRequest&,
  tobas_msgs::GetGnssOriginResponse& res)
{
  if (!gps_fix_)
  {
    res.success = false;
    res.message = "GNSS is not fixed.";
    return true;
  }

  res.latitude = lat_0_;
  res.longitude = lon_0_;

  res.success = true;
  return true;
}

bool ErrorStateKalmanFilterRos::setGnssOriginCb(
  tobas_msgs::SetGnssOriginRequest& req,
  tobas_msgs::SetGnssOriginResponse& res)
{
  if (!gps_fix_)
  {
    res.success = false;
    res.message = "GNSS is not fixed.";
    return true;
  }

  lat_0_ = req.latitude;
  lon_0_ = req.longitude;

  res.success = true;
  return true;
}

void ErrorStateKalmanFilterRos::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  grav_cov_.diagonal().fill(cfg.gravity_variance);
  yaw_var_ = cfg.yaw_variance;
  acc_bias_noise_var_ = do_acc_bias_estimation_ ? exp10(cfg.acc_bias_noise_var_log10) : 0;
  gyro_bias_noise_var_ = do_gyro_bias_estimation_ ? exp10(cfg.gyro_bias_noise_var_log10) : 0;
  grav_noise_var_ = do_grav_estimation_ ? exp10(cfg.gravity_noise_var_log10) : 0;

  TOBAS_INFO("New dynamic parameters are set.");
}
}  // namespace state_estimation_eskf
