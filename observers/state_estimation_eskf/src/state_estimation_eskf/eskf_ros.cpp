#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/geometry.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_std_tools/boost.hpp>
#include <dh_eigen_tools/geometry.hpp>
#include <dh_eigen_tools/iostream.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <static_state_determination/common.hpp>

#include "../../include/state_estimation_eskf/eskf_ros.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;

namespace state_estimation_eskf
{
ErrorStateKalmanFilterRos::ErrorStateKalmanFilterRos()
  : super(),
    is_initialized_(false),
    imu_received_(false),
    mag_received_(false),
    bar_received_(false),
    gps_received_(false),
    vel_received_(false),
    check_topics_timer_(nh_, kTimerPeriod, &ErrorStateKalmanFilterRos::checkTopicsTimerCb, this),
    ac_(static_state_determination::kActionName)
{
  getRosParams();

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

  dh_ros::getParam("~use_gps", use_gps_, kDefaultUseGps);
  dh_ros::getParam(
    "~gps_position_stddev_threshold", gps_pos_stddev_thr_, kDefaultGpsPositionStddevThreshold,
    dh_ros::POSITIVE);

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

  if (use_gps_)
  {
    gps_sub_ = nh_.subscribe("gps", 1, &ErrorStateKalmanFilterRos::gpsCb, this);
    vel_sub_ = nh_.subscribe("ground_speed", 1, &ErrorStateKalmanFilterRos::velCb, this);
  }
}

bool ErrorStateKalmanFilterRos::isReady()
{
  bool ok = true;

  ok &= imu_received_;
  ok &= mag_received_;
  ok &= bar_received_;

  if (use_gps_)
  {
    ok &= gps_received_;
    ok &= vel_received_;
  }

  return ok;
}

void ErrorStateKalmanFilterRos::initialize(const ros::Time& stamp)
{
  rosInfo(
    "Waiting for action server '" << static_state_determination::kActionName << "' to start.");
  ac_.waitForServer();

  rosInfo(
    "Action server '" << static_state_determination::kActionName << "' started, sending goal.");
  static_state_determination::StaticStateDeterminationGoal goal;
  goal.gps_position_stddev_threshold = gps_pos_stddev_thr_;
  ac_.sendGoal(goal);

  bool finished_before_timeout = ac_.waitForResult();
  if (!finished_before_timeout)
  {
    rosthrow("Action did not finish before timeout.");
  }

  const auto result = ac_.getResult();
  rosInfo(
    "The result of " << static_state_determination::kActionName << ":\n"
                     << "IMU count: " << result->imu_count << endl
                     << "Magnetometer count: " << result->mag_count << endl
                     << "Barometer count: " << result->bar_count << endl
                     << "GPS position count: " << result->gps_count << endl
                     << "GPS velocity count: " << result->vel_count << endl
                     << result->imu << endl
                     << result->magnetic_field << result->air_pressure << result->gps
                     << result->ground_speed);

  // 経緯度
  lat_0_ = result->gps.latitude;
  lon_0_ = result->gps.longitude;

  // 高度
  alt_0_ = pressureToAltitude(result->air_pressure.fluid_pressure);

  // 初期姿勢
  tf::vectorMsgToEigen(result->imu.linear_acceleration, a_m_);
  tf::vectorMsgToEigen(result->magnetic_field.magnetic_field, mag_m_);
  const Vector3d m0(ref_mag_north_, -ref_mag_east_, -ref_mag_down_);  // NED -> NWU
  Quaterniond init_q;
  eigen_tools::imuToQuaternion(a_m_, mag_m_, m0, init_q);

  // ISKFを初期化
  // 完全な停止状態で起動するため初期状態の不確かさはかなり小さい想定
  eskf_.initialize(
    acc_noise_density_,                                        // accelerometer noise density
    gyro_noise_density_,                                       // gyrometer noise density
    acc_random_walk_,                                          // accelerometer random walk
    gyro_random_walk_,                                         // gyrometer random walk
    Vector3d(0., 0., -gravity_),                               // gravity vector
    Vector3d(ref_mag_north_, -ref_mag_east_, -ref_mag_down_),  // magnetic field (NWU)
    Vector3d::Zero(),                                          // init position
    Vector3d::Zero(),                                          // init velocity
    init_q,                                                    // init quaternion
    Matrix3d::Zero(),                                          // init position cov
    Matrix3d::Zero(),                                          // init velocity cov
    Matrix3d::Zero(),                                          // init quaternion cov
    Matrix3d::Zero(),                                          // init accelerometer bias cov
    Matrix3d::Zero()                                           // init gyrometer bias cov
  );

  // ヨー角の初期値など
  double roll, pitch;
  quaternionToEuler(init_q.x(), init_q.y(), init_q.z(), init_q.w(), roll, pitch, yaw_prev_);
  yaw_jump_count_ = 0;

  t_last_ = stamp;
}

void ErrorStateKalmanFilterRos::updateBaseStateMsg(const ros::Time& stamp)
{
  // Time stamp
  state_.header.stamp = stamp;

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
  if (!imu_received_)
  {
    imu_received_ = true;
  }

  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      initialize(imu.header.stamp);
      t_ready_ = ros::Time::now();
      is_initialized_ = true;
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
    updateBaseStateMsg(imu.header.stamp);
    posevel_pub_.publish(state_);
  }
}

void ErrorStateKalmanFilterRos::magCb(const MagMsg& mag)
{
  if (!mag_received_)
  {
    mag_received_ = true;
  }

  if (!is_initialized_)
  {
    return;
  }

  tf::vectorMsgToEigen(mag.magnetic_field, mag_m_);
  eskf_.measureMagneticField(mag_m_, rot_mag_cov_);
}

void ErrorStateKalmanFilterRos::barCb(const BarMsg& bar)
{
  if (!bar_received_)
  {
    bar_received_ = true;
  }

  if (!is_initialized_)
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
  if (!gps_received_)
  {
    gps_received_ = true;
  }

  if (!is_initialized_)
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
  if (!vel_received_)
  {
    vel_received_ = true;
  }

  if (!is_initialized_)
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
  if (!imu_received_)
  {
    rosWarn("IMU data is not received yet.");
  }

  if (!mag_received_)
  {
    rosWarn("Magnetometer data is not received yet.");
  }

  if (!bar_received_)
  {
    rosWarn("Barometer data is not received yet.");
  }

  if (use_gps_)
  {
    if (!gps_received_)
    {
      rosWarn("GPS position data is not received yet.");
    }

    if (!vel_received_)
    {
      rosWarn("GPS velocity data is not received yet.");
    }
  }
}

void ErrorStateKalmanFilterRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t level)
{
  rot_acc_cov_.diagonal().fill(cfg_.rotation_variance_grav);
  rot_mag_cov_.diagonal().fill(cfg_.rotation_variance_geomag);
}
}  // namespace state_estimation_eskf
