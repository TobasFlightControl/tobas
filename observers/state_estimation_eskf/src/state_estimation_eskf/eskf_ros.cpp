#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/geometry.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/state_estimation_eskf/eskf_ros.hpp"

#define I_3 (Matrix3d::Identity())
#define TIMER_PERIOD 5.  // [s]
#define WAIT_TO_PUBLISH 3.  // 状態を安定させるためにESKFが稼働してから少し待つ (効果微妙) [s]

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
    vel_subscribed_(false),
    check_topics_timer_(nh_, TIMER_PERIOD, &ErrorStateKalmanFilterRos::checkTopicsTimerCb, this)
{
  getRosParams();
  registerPublishers();
  registerSubscribers();
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
  const Vector3d m0(ref_mag_north_, -ref_mag_east_, -ref_mag_down_);  // NED -> NWU
  const double roll = 0., pitch = 0.;  // 最初は水平状態を想定
  const double yaw = atan2(m0.y() * m.x - m0.x() * m.y, m0.x() * m.x + m0.y() * m.y);
  eulerToQuaternion(roll, pitch, yaw, q_m_.x(), q_m_.y(), q_m_.z(), q_m_.w());
  cout << "Initial quaternion: " << q_m_.coeffs() << endl;

  // ISKFを初期化
  const double freq = 100.;  // テキトー [Hz]
  const auto var_acc = sqr(acc_noise_density_) * freq;
  const auto var_gyro = sqr(gyro_noise_density_) * freq;
  const auto var_acc_bias = sqr(acc_random_walk_) * freq;
  const auto var_gyro_bias = sqr(gyro_random_walk_) * freq;
  eskf_.initialize(
    gravity_,                                          // gravity
    var_acc,                                           // acc variance
    var_gyro,                                          // gyro variance
    var_acc_bias,                                      // acc bias variance
    var_gyro_bias,                                     // gyro bias variance
    Vector3d::Zero(),                                  // init position
    Vector3d::Zero(),                                  // init velocity
    q_m_,                                              // init quaternion
    Vector3d(sqr(3.), sqr(3.), sqr(1.)).asDiagonal(),  // init position cov
    sqr(0.1) * I_3,                                    // init velocity cov
    sqr(1.) * I_3,                                     // init quaternion cov
    var_acc_bias * I_3,                                // init acc bias cov
    var_gyro_bias * I_3                                // init gyro bias cov
  );

  t_last_ = imu_.header.stamp;
}

void ErrorStateKalmanFilterRos::updatePoseVelMsg()
{
  // Time stamp
  state_.header.stamp = imu_.header.stamp;

  // Position
  tf::vectorEigenToKDL(eskf_.getPosition3D(), state_.pose.pos);

  // Rotation
  const auto q = eskf_.getQuaternion();
  auto& rpy = state_.pose.euler;
  quaternionToEuler(q.x(), q.y(), q.z(), q.w(), rpy.roll, rpy.pitch, rpy.yaw);
  cout << "Quaternion: " << q.coeffs() << endl;
  cout << "Euler: " << rpy << endl;

  // Linear velocity (Local)
  tf::vectorEigenToKDL(eskf_.getVelocity(), state_.twist.vel);
  state_.twist.vel = state_.pose.euler.Inverse(state_.twist.vel);  // World -> Local

  // Angular velocity (Local)
  const auto w = w_m_ - eskf_.getGyroBias();
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
      t_ready_ = ros::Time::now();
      is_ready_ = true;
      rosInfo(
        "State estimator is ready. Wait to publish states for " << WAIT_TO_PUBLISH << " seconds.");
    }
    return;
  }

  const auto dt = (imu.header.stamp - t_last_).toSec();
  t_last_ = imu.header.stamp;
  ROS_ASSERT(dt > 0.);

  tf::vectorMsgToEigen(imu.linear_acceleration, a_m_);
  tf::vectorMsgToEigen(imu.angular_velocity, w_m_);

  eskf_.predictIMU(a_m_, w_m_, dt);

  if ((ros::Time::now() - t_ready_).toSec() > WAIT_TO_PUBLISH)
  {
    updatePoseVelMsg();
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

  const Vector3d a = a_m_;
  // const Vector3d a = a_m_ - eskf_.getAccelBias();  // 観測が状態に依存してるのはまずい？
  const auto& m = mag.magnetic_field;
  imuToQuaternion(
    a.x(), a.y(), a.z(), m.x, m.y, m.z, ref_mag_north_, -ref_mag_east_, -ref_mag_down_, q_m_.x(),
    q_m_.y(), q_m_.z(), q_m_.w());

  // TODO: 加速度センサのノイズの分散からクォータニオンのノイズの共分散を正しく計算する
  // sensor_msgs::Imuのlinear_acceleration_covarianceを用いる
  const auto acc_noise_var = sqr(acc_noise_density_) * 1000.;
  const auto quat_var = acc_noise_var / sqr(gravity_);  // これはテキトーにスケーリングしてるだけ

  eskf_.measureQuaternion(q_m_, quat_var * I_3);
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

  double z_m = z_abs - alt_0_;
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

  eskf_.measurePosition2D(xy_m_, cov);
}

void ErrorStateKalmanFilterRos::velCb(const VelMsg& vel)
{
  vel_subscribed_ = true;
  vel_ = vel;

  if (!is_ready_)
  {
    return;
  }

  tf::vectorKDLToEigen(vel.vel, v_m_);

  auto cov_copy = vel.covariance;
  Matrix3d cov = Map<Matrix3d>(cov_copy.data());

  eskf_.measureVelocity(v_m_, cov);
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
