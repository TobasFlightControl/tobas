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
    imu_subscribed_(false),
    mag_subscribed_(false),
    bar_subscribed_(false),
    gps_subscribed_(false),
    vel_subscribed_(false),
    check_topics_timer_(nh_, kTimerPeriod, &ErrorStateKalmanFilterRos::checkTopicsTimerCb, this)
{
  getRosParams();

  yaw_now_ = 0.;
  yaw_prev_ = 0.;
  yaw_jump_count_ = 0;

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
  dh_ros::getParam("/gravity", gravity_);
  dh_ros::getParam("/geomagnetism/north", ref_mag_north_);
  dh_ros::getParam("/geomagnetism/east", ref_mag_east_);
  dh_ros::getParam("/geomagnetism/down", ref_mag_down_);

  dh_ros::getParam("~gyro_noise_density", gyro_noise_density_);
  dh_ros::getParam("~gyro_random_walk", gyro_random_walk_);
  dh_ros::getParam("~acc_noise_density", acc_noise_density_);
  dh_ros::getParam("~acc_random_walk", acc_random_walk_);

  // Dynamic parameters
  dh_ros::getParam("~rotation_variance_acc", cfg_.rotation_variance_acc);
  dh_ros::getParam("~rotation_variance_mag", cfg_.rotation_variance_mag);
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

  ok &= imu_subscribed_;
  ok &= mag_subscribed_;
  ok &= bar_subscribed_;
  ok &= gps_subscribed_;
  ok &= vel_subscribed_;

  return ok;
}

void ErrorStateKalmanFilterRos::setZeroPositions()
{
  // TODO: state_estimation_cascadeのようにバッファの平均をとる？
  lat_0_ = gps_.latitude;
  lon_0_ = gps_.longitude;
  alt_0_ = pressureToAltitude(bar_.fluid_pressure);
}

void ErrorStateKalmanFilterRos::initialize()
{
  setZeroPositions();

  // 地磁気センサから初期姿勢を推定
  const auto& m = mag_.magnetic_field;
  const Vector3d m_0(ref_mag_north_, -ref_mag_east_, -ref_mag_down_);  // NED -> NWU
  const double roll = 0., pitch = 0.;  // 最初は水平状態を想定
  const double yaw = atan2(m_0.y() * m.x - m_0.x() * m.y, m_0.x() * m.x + m_0.y() * m.y);

  Quaterniond q_0;
  eulerToQuaternion(roll, pitch, yaw, q_0.x(), q_0.y(), q_0.z(), q_0.w());
  // std::cout << "Initial quaternion: " << q_0.coeffs() << endl;

  // ISKFを初期化
  const double freq = 100.;  // テキトー [Hz]
  const double var_acc = sqr(acc_noise_density_) * freq;
  const double var_gyro = sqr(gyro_noise_density_) * freq;
  const double var_acc_bias = sqr(acc_random_walk_) * freq;
  const double var_gyro_bias = sqr(gyro_random_walk_) * freq;
  eskf_.initialize(
    var_acc,                                                   // acc variance
    var_gyro,                                                  // gyro variance
    var_acc_bias,                                              // acc bias variance
    var_gyro_bias,                                             // gyro bias variance
    Vector3d(0., 0., -gravity_),                               // gravity vector
    Vector3d(ref_mag_north_, -ref_mag_east_, -ref_mag_down_),  // magnetic field (NWU)
    Vector3d::Zero(),                                          // init position
    Vector3d::Zero(),                                          // init velocity
    q_0,                                                       // init quaternion
    Vector3d(sqr(3.), sqr(3.), sqr(1.)).asDiagonal(),          // init position cov
    sqr(1.) * I3,                                              // init velocity cov
    1000. * I3,                                                // init quaternion cov  // TODO
    var_acc_bias * I3,                                         // init acc bias cov
    var_gyro_bias * I3                                         // init gyro bias cov
  );

  t_last_ = imu_.header.stamp;
}

void ErrorStateKalmanFilterRos::updateBaseStateMsg()
{
  // Time stamp
  state_.header.stamp = imu_.header.stamp;

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
  imu_subscribed_ = true;
  imu_ = imu;

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
  mag_subscribed_ = true;
  mag_ = mag;

  if (!is_ready_)
  {
    return;
  }

  tf::vectorMsgToEigen(mag.magnetic_field, mag_m_);
  eskf_.measureMagneticField(mag_m_, rot_mag_cov_);
}

void ErrorStateKalmanFilterRos::barCb(const BarMsg& bar)
{
  bar_subscribed_ = true;
  bar_ = bar;

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
  gps_subscribed_ = true;
  gps_ = gps;

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
  vel_subscribed_ = true;
  vel_ = vel;

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
  if (!imu_subscribed_)
  {
    rosWarn("IMU data is not received yet.");
  }

  if (!mag_subscribed_)
  {
    rosWarn("Magnetometer data is not received yet.");
  }

  if (!bar_subscribed_)
  {
    rosWarn("Barometer data is not received yet.");
  }

  if (!gps_subscribed_)
  {
    rosWarn("GPS position data is not received yet.");
  }

  if (!vel_subscribed_)
  {
    rosWarn("GPS velocity data is not received yet.");
  }
}

void ErrorStateKalmanFilterRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t level)
{
  rot_acc_cov_.diagonal().fill(cfg_.rotation_variance_acc);
  rot_mag_cov_.diagonal().fill(cfg_.rotation_variance_mag);
}
}  // namespace state_estimation_eskf
