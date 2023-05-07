#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/geometry.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/state_estimation_eskf/eskf_ros.hpp"

#define TIMER_PERIOD 5.
#define I_3 (Matrix3d::Identity())

using namespace std;
using namespace Eigen;
using namespace dh_std;

ErrorStateKalmanFilterRos::ErrorStateKalmanFilterRos()
  : super(),
    is_ready_(false),
    imu_subscribed_(false),
    mag_subscribed_(false),
    bar_subscribed_(false),
    gps_subscribed_(false),
    vel_subscribed_(false)
{
  getRosParams();
  registerPublishers();
  registerSubscribers();
  createTimers();
}

void ErrorStateKalmanFilterRos::getRosParams()
{
  dh_ros::getParam("/gravity", gravity_);
  dh_ros::getParam("/geomagnetism/north", ref_mag_.x());
  dh_ros::getParam("/geomagnetism/east", ref_mag_.y());
  dh_ros::getParam("/geomagnetism/down", ref_mag_.z());

  dh_ros::getParam("~gyro_noise_density", gyro_noise_density_);
  dh_ros::getParam("~gyro_random_walk", gyro_random_walk_);
  dh_ros::getParam("~acc_noise_density", acc_noise_density_);
  dh_ros::getParam("~acc_random_walk", acc_random_walk_);
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

void ErrorStateKalmanFilterRos::createTimers()
{
  check_topics_timer_ = nh_.createTimer(
    ros::Duration(TIMER_PERIOD), &ErrorStateKalmanFilterRos::checkTopicsTimerCb, this);
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

  const auto& a = imu_.linear_acceleration;
  const auto& m = mag_.magnetic_field;
  imuToQuaternion(
    a.x, a.y, a.z, m.x, m.y, m.z, ref_mag_.x(), ref_mag_.y(), ref_mag_.z(), q_m_.x(), q_m_.y(),
    q_m_.z(), q_m_.w());

  // TODO: eskf_.initializeをstate_estimation_cascadeのようにする
  eskf_.initialize(
    Vector3d(0, 0, -gravity_),  // Acceleration due to gravity in global frame
    ErrorStateKalmanFilter::makeState(
      Vector3d::Zero(),         // init pos
      Vector3d::Zero(),         // init vel
      q_m_,                     // init quaternion
      Vector3d::Zero(),         // init accel bias
      Vector3d::Zero()          // init gyro bias
      ),
    ErrorStateKalmanFilter::makeP(
      sqr(1.) * I_3, sqr(0.1) * I_3, sqr(1.) * I_3, sqr(10 * 0.001 * 0.00124) * I_3,
      sqr(10 * 0.001 * 0.276) * I_3),
    sqr(0.00124), sqr(0.276), sqr(0.001 * 0.00124), sqr(0.001 * 0.276),
    ErrorStateKalmanFilter::DelayType::APPLY_UPDATE_TO_NEW, 100);  // TODO: 他の手法も試してみる

  t_last_ = ros::Time::now();
}

void ErrorStateKalmanFilterRos::updatePoseVelMsg()
{
  state_.header.stamp = ros::Time::now();

  tf::vectorEigenToKDL(eskf_.getPosition3D(), state_.pose.pos);

  auto q = eskf_.getQuaternion();
  auto& rpy = state_.pose.euler;
  quaternionToEuler(q.x(), q.y(), q.z(), q.w(), rpy.roll, rpy.pitch, rpy.yaw);

  tf::vectorEigenToKDL(eskf_.getVelocity(), state_.twist.vel);

  // 角速度だけはローカル座標系
  Vector3d w = w_m_ - eskf_.getGyroBias();
  tf::vectorEigenToKDL(w, state_.twist.rot);
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
      is_ready_ = true;
      dh_ros::rosInfo("State estimator is ready.");
    }
    return;
  }

  ros::Duration diff = imu.header.stamp - t_last_;
  t_last_ = imu.header.stamp;
  ROS_ASSERT(diff.toSec() > 0.);
  lTime stamp(imu.header.stamp.sec, imu.header.stamp.nsec);

  tf::vectorMsgToEigen(imu.linear_acceleration, a_m_);
  tf::vectorMsgToEigen(imu.angular_velocity, w_m_);

  eskf_.predictIMU(a_m_, w_m_, diff.toSec(), stamp);
  // eskf_.predictIMU(Vector3d(0, 0, gravity_), Vector3d::Zero(), diff.toSec(), stamp);

  updatePoseVelMsg();
  posevel_pub_.publish(state_);
}

void ErrorStateKalmanFilterRos::magCb(const MagMsg& mag)
{
  mag_subscribed_ = true;
  mag_ = mag;

  if (!is_ready_)
  {
    return;
  }

  lTime stamp(mag.header.stamp.sec, mag.header.stamp.nsec);
  lTime now = getNow();

  // FIXME: 観測が状態に依存してるのはマズい気がする
  const Vector3d a = a_m_ - eskf_.getAccelBias();
  const geometry_msgs::Vector3& m = mag.magnetic_field;
  imuToQuaternion(
    a.x(), a.y(), a.z(), m.x, m.y, m.z, ref_mag_.x(), ref_mag_.y(), ref_mag_.z(), q_m_.x(),
    q_m_.y(), q_m_.z(), q_m_.w());

  // TODO: 加速度センサのノイズの分散からクォータニオンのノイズの共分散を正しく計算する
  // sensor_msgs::Imuのlinear_acceleration_covarianceを用いる
  const double acc_noise_var = sqr(acc_noise_density_) * 1000.;
  const double quat_var = acc_noise_var / sqr(gravity_);  // これはテキトーにスケーリングしてるだけ

  eskf_.measureQuaternion(q_m_, quat_var * I_3, stamp, now);
  // eskf_.measureQuaternion(Quaterniond::Identity(), quat_var * I_3, stamp, now);
}

void ErrorStateKalmanFilterRos::barCb(const BarMsg& bar)
{
  bar_subscribed_ = true;
  bar_ = bar;

  if (!is_ready_)
  {
    return;
  }

  lTime stamp(bar.header.stamp.sec, bar.header.stamp.nsec);
  lTime now = getNow();

  double z_abs, z_var;
  pressureToAltitude(bar.fluid_pressure, bar.variance, z_abs, z_var);

  double z_m = z_abs - alt_0_;
  eskf_.measureAltitude(z_m, z_var, stamp, now);
  // eskf_.measureAltitude(0., z_var, stamp, now);
}

void ErrorStateKalmanFilterRos::gpsCb(const GpsMsg& gps)
{
  gps_subscribed_ = true;
  gps_ = gps;

  if (!is_ready_)
  {
    return;
  }

  lTime stamp(gps.header.stamp.sec, gps.header.stamp.nsec);
  lTime now = getNow();

  gpsToCartRelative(gps.latitude, gps.longitude, lat_0_, lon_0_, xy_m_.x(), xy_m_.y());

  Matrix2d cov;
  cov(0, 0) = gps.position_covariance[0];
  cov(0, 1) = gps.position_covariance[1];
  cov(1, 0) = gps.position_covariance[3];
  cov(1, 1) = gps.position_covariance[4];

  eskf_.measurePosition2D(xy_m_, cov, stamp, now);
  // eskf_.measurePosition2D(Vector2d::Zero(), cov, stamp, now);
}

void ErrorStateKalmanFilterRos::velCb(const VelMsg& vel)
{
  vel_subscribed_ = true;
  vel_ = vel;

  if (!is_ready_)
  {
    return;
  }

  lTime stamp(vel.header.stamp.sec, vel.header.stamp.nsec);
  lTime now = getNow();

  tf::vectorKDLToEigen(vel.vel, v_m_);

  boost::array<double, 9> cov_copy = vel.covariance;
  Matrix3d cov = Map<Matrix3d>(cov_copy.data());

  eskf_.measureVelocity(v_m_, cov, stamp, now);
  // eskf_.measureVelocity(Vector3d::Zero(), cov, stamp, now);
}

void ErrorStateKalmanFilterRos::checkTopicsTimerCb(const ros::TimerEvent& event)
{
  if (!imu_subscribed_)
  {
    dh_ros::rosWarn("IMU data is not received yet.");
  }

  if (!mag_subscribed_)
  {
    dh_ros::rosWarn("Magnetometer data is not received yet.");
  }

  if (!bar_subscribed_)
  {
    dh_ros::rosWarn("Barometer data is not received yet.");
  }

  if (!gps_subscribed_)
  {
    dh_ros::rosWarn("GPS position data is not received yet.");
  }

  if (!vel_subscribed_)
  {
    dh_ros::rosWarn("GPS velocity data is not received yet.");
  }
}

lTime ErrorStateKalmanFilterRos::getNow()
{
  ros::Time now = ros::Time::now();
  return lTime(now.sec, now.nsec);
}
