#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/geometry.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_std_tools/boost.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/state_estimation_cascade/state_estimator.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;

namespace state_estimation_cascade
{
StateEstimator::StateEstimator()
  : super(),
    is_initialized_(false),
    yaw_now_(0.),
    yaw_prev_(0.),
    yaw_jump_count_(0),
    check_topics_timer_(nh_, kTimerPeriod, &StateEstimator::checkTopicsTimerCb, this)
{
  getRosParams();

  imu_buf_.resize(imu_buf_size_);
  bar_buf_.resize(bar_buf_size_);
  gps_buf_.resize(gps_buf_size_);
  vel_buf_.resize(vel_buf_size_);

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&StateEstimator::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void StateEstimator::getRosParams()
{
  dh_ros::getParam("/gravity", gravity_, dh_ros::POSITIVE);

  dh_ros::getParam("~use_gps", use_gps_, kDefaultUseGps);
  dh_ros::getParam("~imu_buf_size", imu_buf_size_, kDefaultImuBufSize, dh_ros::POSITIVE);
  dh_ros::getParam("~bar_buf_size", bar_buf_size_, kDefaultBarBufSize, dh_ros::POSITIVE);
  dh_ros::getParam("~gps_buf_size", gps_buf_size_, kDefaultGpsBufSize, dh_ros::POSITIVE);
  dh_ros::getParam("~vel_buf_size", vel_buf_size_, kDefaultVelBufSize, dh_ros::POSITIVE);

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
  if (!imu_buf_.isFull())
  {
    return false;
  }
  if (!bar_buf_.isFull())
  {
    return false;
  }

  if (use_gps_)
  {
    if (!gps_buf_.isFull())
    {
      return false;
    }
    if (!vel_buf_.isFull())
    {
      return false;
    }
  }

  return true;
}

void StateEstimator::initialize()
{
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

  t_last_ = imu_buf_.getLatest().header.stamp;
}

void StateEstimator::setZeroPositions()
{
  // 緯度，経度 [deg]
  if (use_gps_)
  {
    double sum_lat = 0.;
    double sum_lon = 0.;
    for (int i = 0; i < gps_buf_size_; ++i)
    {
      const auto& gps = gps_buf_.get(i);
      sum_lat += gps.latitude;
      sum_lon += gps.longitude;
    }
    lat_0_ = sum_lat / gps_buf_size_;
    lon_0_ = sum_lon / gps_buf_size_;
  }

  // 高度 [m]
  double sum_pressure = 0.;
  for (int i = 0; i < bar_buf_size_; ++i)
  {
    const auto& bar = bar_buf_.get(i);
    sum_pressure += bar.fluid_pressure;
  }
  const double mean_pressure = sum_pressure / bar_buf_size_;
  alt_0_ = pressureToAltitude(mean_pressure);
}

void StateEstimator::updatePoseVelMsg()
{
  const auto& imu = imu_buf_.getLatest();

  // Time stamp
  state_.header.stamp = imu.header.stamp;

  // Position
  tf::vectorEigenToKDL(cart_filter_.getXYZ(), state_.pose.pos);

  // Roll, Pitch
  const auto& quat = imu_buf_.getLatest().orientation;
  auto& rpy = state_.pose.euler;
  quaternionToEuler(quat.x, quat.y, quat.z, quat.w, rpy.roll, rpy.pitch, yaw_now_);

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

void StateEstimator::filteredImuCb(const ImuMsg& imu)
{
  imu_buf_.add(imu);

  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      initialize();
      is_initialized_ = true;
      rosInfo("State estimator is ready.");
    }
    return;
  }

  const double dt = (imu.header.stamp - t_last_).toSec();
  ROS_ASSERT(dt >= 0.);
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
  updatePoseVelMsg();
  posevel_pub_.publish(state_);
}

void StateEstimator::barometerCb(const BarMsg& bar)
{
  bar_buf_.add(bar);

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
  gps_buf_.add(gps);

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
  vel_buf_.add(vel);

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
  if (imu_buf_.isEmpty())
  {
    rosWarn("Filtered IMU data is not received yet.");
  }
  else if (!imu_buf_.isFull())
  {
    rosInfoOnce("Waiting for Filtered IMU data to be collected.");
  }

  // Barometer
  if (bar_buf_.isEmpty())
  {
    rosWarn("Barometer data is not received yet.");
  }
  else if (!bar_buf_.isFull())
  {
    rosInfoOnce("Waiting for Barometer data to be collected.");
  }

  if (use_gps_)
  {
    // GPS position
    if (gps_buf_.isEmpty())
    {
      rosWarn("GPS position data is not received yet.");
    }
    else if (!gps_buf_.isFull())
    {
      rosInfoOnce("Waiting for GPS position data to be collected.");
    }

    // GPS velocity
    if (vel_buf_.isEmpty())
    {
      rosWarn("GPS velocity data is not received yet.");
    }
    else if (!vel_buf_.isFull())
    {
      rosInfoOnce("Waiting for GPS velocity data to be collected.");
    }
  }
}

void StateEstimator::dynamicReconfigureCb(const ConfigType& cfg, uint32_t level)
{
  cart_filter_.reconfigure(cfg.gravity_variance);
}
}  // namespace state_estimation_cascade
