#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/geometry.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_std_tools/boost.hpp>
#include <dh_std_tools/exception.hpp>
#include <dh_eigen_tools/geometry.hpp>
#include <dh_eigen_tools/iostream.hpp>
#include <dh_eigen_tools/conversion/eigen_boost.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <static_state_determination/StaticStateDeterminationAction.h>
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
    check_topics_timer_(nh_, kTimerPeriod, &ErrorStateKalmanFilterRos::checkTopicsTimerCb, this)
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

  dh_ros::getParam("~use_barometer", use_bar_, kDefaultUseBarometer);
  dh_ros::getParam("~use_gps", use_gps_, kDefaultUseGps);
  dh_ros::getParam(
    "~gps_position_stddev_threshold", gps_pos_stddev_thr_, kDefaultGpsPositionStddevThreshold,
    dh_ros::POSITIVE);

  string geomag_observe_method;
  dh_ros::getParam("~geomag_observe_method", geomag_observe_method, kDefaultGeomagObserveMethod);
  if (geomag_observe_method == "rpy")
  {
    geomag_observe_method_ = GeomagObserveMethod::RPY;
  }
  else if (geomag_observe_method == "yaw_only")
  {
    geomag_observe_method_ = GeomagObserveMethod::YAW_ONLY;
  }
  else
  {
    rosthrow("Invalid geomagnetism observation method: " << geomag_observe_method);
  }

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
  event_sub_ = nh_.subscribe("event", 1, &ErrorStateKalmanFilterRos::eventCb, this);
  imu_sub_ = nh_.subscribe("imu", 1, &ErrorStateKalmanFilterRos::imuCb, this);
  mag_sub_ = nh_.subscribe("magnetic_field", 1, &ErrorStateKalmanFilterRos::magCb, this);

  if (use_bar_)
  {
    bar_sub_ = nh_.subscribe("air_pressure", 1, &ErrorStateKalmanFilterRos::barCb, this);
  }

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

  if (use_bar_)
  {
    ok &= bar_received_;
  }

  if (use_gps_)
  {
    ok &= gps_received_;
    ok &= vel_received_;
  }

  return ok;
}

void ErrorStateKalmanFilterRos::initialize(const ros::Time& stamp)
{
  // 初期化処理の開始時間
  ros::Time start_time = ros::Time::now();

  // 静止状態でのセンサデータを平均してゼロ点を決める
  setZeroPositions();

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
    q_0_,                                                      // init quaternion
    Matrix3d::Zero(),                                          // init position cov
    Matrix3d::Zero(),                                          // init velocity cov
    Matrix3d::Zero(),                                          // init quaternion cov
    Matrix3d::Zero(),                                          // init accelerometer bias cov
    Matrix3d::Zero()                                           // init gyrometer bias cov
  );

  // ヨー角の初期値
  double roll, pitch;
  quaternionToEuler(q_0_.x(), q_0_.y(), q_0_.z(), q_0_.w(), roll, pitch, yaw_prev_);

  yaw_jump_count_ = 0;

  // IMUのタイムスタンプに初期化に要した時間を足した時間を最新のセンサ時間とする
  const auto duration = ros::Time::now() - start_time;
  t_last_ = t_ready_ = stamp + duration;
}

void ErrorStateKalmanFilterRos::setZeroPositions()
{
  actionlib::SimpleActionClient<static_state_determination::StaticStateDeterminationAction> ac(
    static_state_determination::kActionName);
  rosInfo(
    "Waiting for action server '" << static_state_determination::kActionName << "' to start.");
  ac.waitForServer();

  rosInfo(
    "Action server '" << static_state_determination::kActionName << "' started, sending goal.");
  static_state_determination::StaticStateDeterminationGoal goal;
  goal.gps_position_stddev_threshold = gps_pos_stddev_thr_;
  ac.sendGoal(goal);

  const bool finished_before_timeout = ac.waitForResult();
  if (!finished_before_timeout)
  {
    rosthrow("Action did not finish before timeout.");
  }

  const auto result = ac.getResult();
  rosInfo(
    "The result of " << static_state_determination::kActionName << ":\n"
                     << "IMU count: " << result->imu_count << endl
                     << "Magnetometer count: " << result->mag_count << endl
                     << "Barometer count: " << result->bar_count << endl
                     << "GPS position count: " << result->gps_count << endl
                     << "GPS velocity count: " << result->vel_count << endl
                     << "IMU:\n"
                     << result->imu << endl
                     << "Magnetic Field:\n"
                     << result->magnetic_field << endl
                     << "Air Pressure:\n"
                     << result->air_pressure << endl
                     << "GPS:\n"
                     << result->gps << endl
                     << "Ground Speed:\n"
                     << result->ground_speed);

  // GPS
  lat_0_ = result->gps.latitude;
  lon_0_ = result->gps.longitude;
  alt_0_gps_ = result->gps.altitude;

  // Barometer
  alt_0_bar_ = pressureToAltitude(result->air_pressure.fluid_pressure);

  // 初期姿勢
  tf::vectorMsgToEigen(result->imu.linear_acceleration, a_m_);
  tf::vectorMsgToEigen(result->magnetic_field.magnetic_field, mag_m_);
  const Vector3d m0(ref_mag_north_, -ref_mag_east_, -ref_mag_down_);  // NED -> NWU
  eigen_tools::imuToQuaternion(a_m_, mag_m_, m0, q_0_);
}

void ErrorStateKalmanFilterRos::updateBaseStateMsg(const ImuMsg& imu)
{
  // Time stamp
  state_.header.stamp = imu.header.stamp;

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

  // Covariances
  eigen_tools::matrix3EigenToBoost(eskf_.getPositionCovariance(), state_.position_covariance);
  eigen_tools::matrix3EigenToBoost(eskf_.getOrientationCovariance(), state_.orientation_covariance);
  eigen_tools::matrix3EigenToBoost(
    eskf_.getVelocityCovariance(), state_.linear_velocity_covariance);
  state_.angular_velocity_covariance = imu.angular_velocity_covariance;  // ジャイロはそのまま

  // For debug
  // std::cout << "Estiamted Quaternion: " << endl << q << endl;
  // std::cout << "Estimated state:" << endl << state_ << endl;
}

bool ErrorStateKalmanFilterRos::isValidImuTimeGap(double dt)
{
  if (dt < 0.)
  {
    rosFatal("The time gap between consecutive IMU sensor readings is negative: " << dt << "[s]");
    return false;
  }

  if (dt > kImuTimeGapThreshold)
  {
    rosFatal(
      "The time gap between consecutive IMU sensor readings " << dt << "[s] exceeds the threshold "
                                                              << kImuTimeGapThreshold << "[s]");
    return false;
  }

  if (dt == 0.)
  {
    rosWarn("The time gap between consecutive IMU sensor readings is zero.");
    return false;
  }

  return true;
}

void ErrorStateKalmanFilterRos::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      ros::shutdown();
      break;
    default:
      break;
  }
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
      is_initialized_ = true;
      rosInfo(
        "State estimator is ready. Wait to publish states for " << kWaitToPublish << " seconds.");
    }
    return;
  }

  const double dt = (imu.header.stamp - t_last_).toSec();
  if (!isValidImuTimeGap(dt))
  {
    return;
  }
  t_last_ = imu.header.stamp;

  tf::vectorMsgToEigen(imu.linear_acceleration, a_m_);
  tf::vectorMsgToEigen(imu.angular_velocity, w_m_);

  // 事前予測
  eskf_.predictIMU(a_m_, w_m_, dt);

  // 重力方向の観測
  eskf_.measureAcceleration(a_m_, rot_acc_cov_);

  // 推定状態を発行
  if ((ros::Time::now() - t_ready_).toSec() > kWaitToPublish)
  {
    updateBaseStateMsg(imu);
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

  switch (geomag_observe_method_)
  {
    case GeomagObserveMethod::RPY:
      eskf_.measureMagneticFieldRPY(mag_m_, rot_mag_cov_);
      break;
    case GeomagObserveMethod::YAW_ONLY:
      eskf_.measureMagneticFieldYaw(mag_m_.x(), mag_m_.y(), rot_mag_cov_(0, 0));
      break;
    default:
      throw NotImplementedError();
  }
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

  const double z_m = z_abs - alt_0_bar_;
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

  gpsToCartRelative(gps.latitude, gps.longitude, lat_0_, lon_0_, pos_m_.x(), pos_m_.y());
  pos_m_.z() = gps.altitude - alt_0_gps_;
  // cout << "Measured position: " << endl << pos_m_ << endl;

  auto cov_copy = gps.position_covariance;
  Matrix3d cov = Map<Matrix3d>(cov_copy.data());

  eskf_.measureXYZ(pos_m_, cov);
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

  if (use_bar_ && !bar_received_)
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

void ErrorStateKalmanFilterRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  rot_acc_cov_.diagonal().fill(cfg.rotation_variance_grav);
  rot_mag_cov_.diagonal().fill(cfg.rotation_variance_geomag);

  rosInfo("New dynamic parameters are set.");
}
}  // namespace state_estimation_eskf
