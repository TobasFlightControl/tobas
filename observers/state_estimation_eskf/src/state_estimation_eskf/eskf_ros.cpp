#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/geometry.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_eigen_tools/iostream.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/state_estimation_eskf/eskf_ros.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;

namespace state_estimation_eskf
{
ErrorStateKalmanFilterRos::ErrorStateKalmanFilterRos()
  : super(),
    is_ready_(false),
    yaw_now_(0.),
    yaw_prev_(0.),
    yaw_jump_count_(0),
    check_topics_timer_(nh_, kTimerPeriod, &ErrorStateKalmanFilterRos::checkTopicsTimerCb, this)
{
  getRosParams();

  imu_buf_.resize(imu_buf_size_);
  mag_buf_.resize(mag_buf_size_);
  bar_buf_.resize(bar_buf_size_);
  gps_buf_.resize(gps_buf_size_);
  vel_buf_.resize(vel_buf_size_);

  rot_acc_cov_.setZero();
  rot_mag_cov_.setZero();

  dynamicReconfigureCb(cfg_, 0);

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f =
    boost::bind(&ErrorStateKalmanFilterRos::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void ErrorStateKalmanFilterRos::getRosParams()
{
  dh_ros::getParam("/gravity", gravity_, dh_ros::POSITIVE);
  dh_ros::getParam("/geomagnetism/north", ref_mag_north_);
  dh_ros::getParam("/geomagnetism/east", ref_mag_east_);
  dh_ros::getParam("/geomagnetism/down", ref_mag_down_);

  dh_ros::getParam("~gyro_noise_density", gyro_noise_density_, dh_ros::POSITIVE);
  dh_ros::getParam("~gyro_random_walk", gyro_random_walk_, dh_ros::POSITIVE);
  dh_ros::getParam("~acc_noise_density", acc_noise_density_, dh_ros::POSITIVE);
  dh_ros::getParam("~acc_random_walk", acc_random_walk_, dh_ros::POSITIVE);

  dh_ros::getParam("~imu_buf_size", imu_buf_size_, kDefaultImuBufSize, dh_ros::POSITIVE);
  dh_ros::getParam("~mag_buf_size", mag_buf_size_, kDefaultMagBufSize, dh_ros::POSITIVE);
  dh_ros::getParam("~bar_buf_size", bar_buf_size_, kDefaultBarBufSize, dh_ros::POSITIVE);
  dh_ros::getParam("~gps_buf_size", gps_buf_size_, kDefaultGpsBufSize, dh_ros::POSITIVE);
  dh_ros::getParam("~vel_buf_size", vel_buf_size_, kDefaultVelBufSize, dh_ros::POSITIVE);

  // Dynamic parameters
  dh_ros::getParam("~rotation_variance_grav", cfg_.rotation_variance_grav, dh_ros::POSITIVE);
  dh_ros::getParam("~rotation_variance_geomag", cfg_.rotation_variance_geomag, dh_ros::POSITIVE);
}

void ErrorStateKalmanFilterRos::registerPublishers()
{
  posevel_pub_ = nh_.advertise<StateMsg>("base_state", 1);
}

void ErrorStateKalmanFilterRos::registerSubscribers()
{
  imu_sub_ = nh_.subscribe("imu", 1, &ErrorStateKalmanFilterRos::imuCb, this);
  mag_sub_ = nh_.subscribe("magnetic_field", 1, &ErrorStateKalmanFilterRos::magCb, this);
  bar_sub_ = nh_.subscribe("air_pressure", 1, &ErrorStateKalmanFilterRos::barCb, this);
  gps_sub_ = nh_.subscribe("gps", 1, &ErrorStateKalmanFilterRos::gpsCb, this);
  vel_sub_ = nh_.subscribe("ground_speed", 1, &ErrorStateKalmanFilterRos::velCb, this);
}

bool ErrorStateKalmanFilterRos::isReady()
{
  bool ok = true;

  ok &= imu_buf_.isFull();
  ok &= mag_buf_.isFull();
  ok &= bar_buf_.isFull();
  ok &= gps_buf_.isFull();
  ok &= vel_buf_.isFull();

  return ok;
}

void ErrorStateKalmanFilterRos::setZeroPositions()
{
  // 緯度，経度 [deg]
  double sum_lat = 0.;
  double sum_lon = 0.;
  for (int i = 0; i < gps_buf_.maxSize(); ++i)
  {
    const auto& gps = gps_buf_.get(i);
    sum_lat += gps.latitude;
    sum_lon += gps.longitude;
  }
  lat_0_ = sum_lat / gps_buf_.maxSize();
  lon_0_ = sum_lon / gps_buf_.maxSize();

  // 高度 [m]
  double sum_pressure = 0.;
  for (int i = 0; i < bar_buf_.maxSize(); ++i)
  {
    const auto& bar = bar_buf_.get(i);
    sum_pressure += bar.fluid_pressure;
  }
  const double mean_pressure = sum_pressure / bar_buf_.maxSize();
  alt_0_ = pressureToAltitude(mean_pressure);
}

void ErrorStateKalmanFilterRos::initialize()
{
  setZeroPositions();

  // 速度の平均を計算
  Vector3d sum_v = Vector3d::Zero();
  for (int i = 0; i < vel_buf_.maxSize(); ++i)
  {
    const auto& vel = vel_buf_.getLatest();
    sum_v.x() += vel.vel.x();
    sum_v.y() += vel.vel.y();
    sum_v.z() += vel.vel.z();
  }
  const Vector3d v = sum_v / vel_buf_.maxSize();

  // 地磁気の平均から初期姿勢を推定
  Vector3d sum_m = Vector3d::Zero();
  for (int i = 0; i < mag_buf_.maxSize(); ++i)
  {
    const auto& mag = mag_buf_.get(i);
    sum_m.x() += mag.magnetic_field.x;
    sum_m.y() += mag.magnetic_field.y;
    sum_m.z() += mag.magnetic_field.z;
  }
  const Vector3d m = sum_m / mag_buf_.maxSize();  // 比率が問題なので和をそのまま地磁気としてもよい
  const Vector3d m0(ref_mag_north_, -ref_mag_east_, -ref_mag_down_);  // NED -> NWU
  const double roll = 0., pitch = 0.;  // 最初は水平状態を想定
  const double yaw = atan2(m0.y() * m.x() - m0.x() * m.y(), m0.x() * m.x() + m0.y() * m.y());

  // 初期姿勢をクオータニオンに変換
  Quaterniond q;
  eulerToQuaternion(roll, pitch, yaw, q.x(), q.y(), q.z(), q.w());
  // std::cout << "Initial quaternion: " << q.coeffs() << endl;

  // ISKFを初期化
  // 共分散の初期値は想定しうる最大値以上に設定している
  eskf_.initialize(
    acc_noise_density_,                                        // accelerometer noise density
    gyro_noise_density_,                                       // gyrometer noise density
    acc_random_walk_,                                          // accelerometer random walk
    gyro_random_walk_,                                         // gyrometer random walk
    Vector3d(0., 0., -gravity_),                               // gravity vector
    Vector3d(ref_mag_north_, -ref_mag_east_, -ref_mag_down_),  // magnetic field (NWU)
    Vector3d::Zero(),                                          // init position
    v,                                                         // init velocity
    q,                                                         // init quaternion
    sqr(10.) * I3,                                             // init position cov
    sqr(1.) * I3,                                              // init velocity cov
    1000. * I3,                                                // init quaternion cov
    1. * I3,                                                   // init acc bias cov
    1. * I3                                                    // init gyro bias cov
  );

  t_last_ = imu_buf_.getLatest().header.stamp;
}

void ErrorStateKalmanFilterRos::updateBaseStateMsg()
{
  // Time stamp
  state_.header.stamp = imu_buf_.getLatest().header.stamp;

  // Position
  tf::vectorEigenToKDL(eskf_.getXYZ(), state_.pose.pos);

  // Roll, Pitch
  const Quaterniond q = eskf_.getQuaternion();
  auto& rpy = state_.pose.euler;
  quaternionToEuler(q.x(), q.y(), q.z(), q.w(), rpy.roll, rpy.pitch, yaw_now_);

  // Yaw
  if (yaw_now_ - yaw_prev_ > M_PI)  // 負方向のジャンプを検出
  {
    --yaw_jump_count_;
  }
  else if (yaw_now_ - yaw_prev_ < -M_PI)  // 正方向のジャンプを検出
  {
    ++yaw_jump_count_;
  }
  yaw_prev_ = yaw_now_;
  rpy.yaw = (2 * M_PI) * yaw_jump_count_ + yaw_now_;

  // Linear velocity (Local)
  tf::vectorEigenToKDL(eskf_.getVelocity(), state_.twist.vel);
  state_.twist.vel = state_.pose.euler.Inverse(state_.twist.vel);  // World -> Local

  // Angular velocity (Local)
  const Vector3d w = w_m_ - eskf_.getGyroBias();
  tf::vectorEigenToKDL(w, state_.twist.rot);

  // For debug
  // std::cout << "Estiamted Quaternion: " << endl << q << endl;
  // std::cout << "Estimated state:" << endl << state_ << endl;
}

void ErrorStateKalmanFilterRos::imuCb(const ImuMsg& imu)
{
  imu_buf_.add(imu);

  if (!is_ready_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      initialize();
      t_ready_ = ros::Time::now();
      is_ready_ = true;
      rosInfo(
        "State estimator is ready. Wait to publish states for " << kWaitToPublish << " seconds.");
    }
    return;
  }

  const double dt = (imu.header.stamp - t_last_).toSec();
  t_last_ = imu.header.stamp;
  ROS_ASSERT(dt > 0.);

  tf::vectorMsgToEigen(imu.linear_acceleration, a_m_);
  tf::vectorMsgToEigen(imu.angular_velocity, w_m_);

  // 事前予測
  eskf_.predictIMU(a_m_, w_m_, dt);

  // 重力方向の観測
  eskf_.measureAcceleration(a_m_, rot_acc_cov_);

  // 推定状態を発行
  if ((ros::Time::now() - t_ready_).toSec() > kWaitToPublish)
  {
    updateBaseStateMsg();
    posevel_pub_.publish(state_);
  }
}

void ErrorStateKalmanFilterRos::magCb(const MagMsg& mag)
{
  mag_buf_.add(mag);

  if (!is_ready_)
  {
    return;
  }

  tf::vectorMsgToEigen(mag.magnetic_field, mag_m_);
  eskf_.measureMagneticField(mag_m_, rot_mag_cov_);
}

void ErrorStateKalmanFilterRos::barCb(const BarMsg& bar)
{
  bar_buf_.add(bar);

  if (!is_ready_)
  {
    return;
  }

  double z_abs, z_var;
  pressureToAltitude(bar.fluid_pressure, bar.variance, z_abs, z_var);

  const double z_m = z_abs - alt_0_;
  eskf_.measureAltitude(z_m, z_var);
}

void ErrorStateKalmanFilterRos::gpsCb(const GpsMsg& gps)
{
  gps_buf_.add(gps);

  if (!is_ready_)
  {
    return;
  }

  gpsToCartRelative(gps.latitude, gps.longitude, lat_0_, lon_0_, xy_m_.x(), xy_m_.y());

  Matrix2d cov;
  cov(0, 0) = gps.position_covariance[0];
  cov(0, 1) = gps.position_covariance[1];
  cov(1, 0) = gps.position_covariance[3];
  cov(1, 1) = gps.position_covariance[4];

  eskf_.measureXY(xy_m_, cov);
}

void ErrorStateKalmanFilterRos::velCb(const VelMsg& vel)
{
  vel_buf_.add(vel);

  if (!is_ready_)
  {
    return;
  }

  tf::vectorKDLToEigen(vel.vel, vel_m_);

  auto cov_copy = vel.covariance;
  Matrix3d cov = Map<Matrix3d>(cov_copy.data());

  eskf_.measureVelocity(vel_m_, cov);
}

void ErrorStateKalmanFilterRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (imu_buf_.isEmpty())
  {
    rosWarn("IMU data is not received yet.");
  }

  if (mag_buf_.isEmpty())
  {
    rosWarn("Magnetometer data is not received yet.");
  }

  if (bar_buf_.isEmpty())
  {
    rosWarn("Barometer data is not received yet.");
  }

  if (gps_buf_.isEmpty())
  {
    rosWarn("GPS position data is not received yet.");
  }

  if (vel_buf_.isEmpty())
  {
    rosWarn("GPS velocity data is not received yet.");
  }
}

void ErrorStateKalmanFilterRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t level)
{
  rot_acc_cov_.diagonal().fill(cfg_.rotation_variance_grav);
  rot_mag_cov_.diagonal().fill(cfg_.rotation_variance_geomag);
}
}  // namespace state_estimation_eskf
