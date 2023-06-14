#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/geometry.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_std_tools/boost.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <static_state_determination/StaticStateDeterminationAction.h>
#include <static_state_determination/common.hpp>

#include "../../include/state_estimation_cascade/state_estimator.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;

namespace state_estimation_cascade
{
StateEstimator::StateEstimator()
  : super(),
    is_initialized_(false),
    imu_received_(false),
    bar_received_(false),
    gps_received_(false),
    vel_received_(false),
    check_topics_timer_(nh_, kTimerPeriod, &StateEstimator::checkTopicsTimerCb, this)
{
  getRosParams();
  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&StateEstimator::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void StateEstimator::getRosParams()
{
  dh_ros::getParam("/gravity", gravity_, dh_ros::POSITIVE);

  dh_ros::getParam("~use_gps", use_gps_, kDefaultUseGps);
  dh_ros::getParam(
    "~gps_position_stddev_threshold", gps_pos_stddev_thr_, kDefaultGpsPositionStddevThreshold,
    dh_ros::POSITIVE);

  // Dynamic parameters
  dh_ros::getParam("~gravity_variance", grav_var_, dh_ros::POSITIVE);
}

void StateEstimator::registerPublishers()
{
  posevel_pub_ = nh_.advertise<StateMsg>("base_state", 1);
}

void StateEstimator::registerSubscribers()
{
  filtered_imu_sub_ = nh_.subscribe("filtered_imu", 1, &StateEstimator::filteredImuCb, this);
  bar_sub_ = nh_.subscribe("air_pressure", 1, &StateEstimator::barometerCb, this);

  if (use_gps_)
  {
    gps_pos_sub_ = nh_.subscribe("gps", 1, &StateEstimator::gpsPositionCb, this);
    gps_vel_sub_ = nh_.subscribe("ground_speed", 1, &StateEstimator::gpsVelocityCb, this);
  }
}

bool StateEstimator::isReady()
{
  if (!imu_received_)
  {
    return false;
  }
  if (!bar_received_)
  {
    return false;
  }

  if (use_gps_)
  {
    if (!gps_received_)
    {
      return false;
    }
    if (!vel_received_)
    {
      return false;
    }
  }

  return true;
}

void StateEstimator::initialize(const ImuMsg& imu)
{
  // 初期化処理の開始時間
  ros::Time start_time = ros::Time::now();

  // 静止状態でのセンサデータを平均してゼロ点を決める
  setZeroPositions();

  // カルマンフィルタを初期化
  // 完全な停止状態で起動するため初期状態の不確かさはかなり小さい想定
  cart_filter_.initialize(
    Vector3d::Zero(),             // init position
    Vector3d::Zero(),             // init velocity
    Vector3d::Zero(),             // init acceleration without gravity
    Vector3d(0., 0., -gravity_),  // init gravity
    Matrix3d::Zero(),             // init position cov
    Matrix3d::Zero(),             // init velocity cov
    Matrix3d::Zero(),             // init acceleration cov
    Matrix3d::Zero(),             // init gravity cov
    grav_var_                     // gravity variance
  );

  // ヨー角の初期値
  const auto& q0 = imu.orientation;
  double roll, pitch;
  quaternionToEuler(q0.x, q0.y, q0.z, q0.w, roll, pitch, yaw_prev_);

  yaw_jump_count_ = 0;

  // IMUのタイムスタンプに初期化に要した時間を足した時間を最新のセンサ時間とする
  const auto duration = ros::Time::now() - start_time;
  t_last_ = imu.header.stamp + duration;
}

void StateEstimator::setZeroPositions()
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

  // 経緯度
  lat_0_ = result->gps.latitude;
  lon_0_ = result->gps.longitude;

  // 高度
  alt_0_ = pressureToAltitude(result->air_pressure.fluid_pressure);
}

void StateEstimator::updatePoseVelMsg(const ImuMsg& imu)
{
  // Time stamp
  state_.header.stamp = imu.header.stamp;

  // Position
  tf::vectorEigenToKDL(cart_filter_.getXYZ(), state_.pose.pos);

  // Roll, Pitch
  const auto& q = imu.orientation;
  auto& rpy = state_.pose.euler;
  quaternionToEuler(q.x, q.y, q.z, q.w, rpy.roll, rpy.pitch, yaw_now_);

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
  tf::vectorEigenToKDL(cart_filter_.getVelocity(), state_.twist.vel);
  state_.twist.vel = state_.pose.euler.Inverse(state_.twist.vel);  // World -> Local

  // Angular velocity (Local)
  tf::vectorMsgToKDL(imu.angular_velocity, state_.twist.rot);
}

bool StateEstimator::isValidImuTimeGap(double dt)
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

void StateEstimator::filteredImuCb(const ImuMsg& imu)
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
      initialize(imu);
      is_initialized_ = true;
      rosInfo("State estimator is ready.");
    }
    return;
  }

  const double dt = (imu.header.stamp - t_last_).toSec();
  if (!isValidImuTimeGap(dt))
  {
    return;
  }
  t_last_ = imu.header.stamp;

  tf::quaternionMsgToEigen(imu.orientation, quat_);
  tf::vectorMsgToEigen(imu.linear_acceleration, a_m_);

  auto acc_cov_data = imu.linear_acceleration_covariance;
  Matrix3d acc_cov = Map<Matrix3d>(acc_cov_data.data());

  // 公称状態を更新
  cart_filter_.predict(quat_, acc_cov, dt);

  // 加速度の観測
  cart_filter_.measureAcceleration(a_m_, acc_cov);

  // 推定した状態を発行
  updatePoseVelMsg(imu);
  posevel_pub_.publish(state_);
}

void StateEstimator::barometerCb(const BarMsg& bar)
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
  cart_filter_.measureAltitude(z_m, z_var);
}

void StateEstimator::gpsPositionCb(const GpsMsg& gps)
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

  cart_filter_.measureXY(xy_m_, cov);
}

void StateEstimator::gpsVelocityCb(const VelMsg& vel)
{
  if (!vel_received_)
  {
    vel_received_ = true;
  }

  if (!is_initialized_)
  {
    return;
  }

  tf::vectorKDLToEigen(vel.vel, v_m_);

  auto cov_copy = vel.covariance;
  Matrix3d cov = Map<Matrix3d>(cov_copy.data());

  cart_filter_.measureVelocity(v_m_, cov);
}

void StateEstimator::checkTopicsTimerCb(const ros::TimerEvent&)
{
  // IMU
  if (!imu_received_)
  {
    rosWarn("Filtered IMU data is not received yet.");
  }

  // Barometer
  if (!bar_received_)
  {
    rosWarn("Barometer data is not received yet.");
  }

  if (use_gps_)
  {
    // GPS position
    if (!gps_received_)
    {
      rosWarn("GPS position data is not received yet.");
    }

    // GPS velocity
    if (!vel_received_)
    {
      rosWarn("GPS velocity data is not received yet.");
    }
  }
}

void StateEstimator::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  cart_filter_.reconfigure(cfg.gravity_variance);
}
}  // namespace state_estimation_cascade
