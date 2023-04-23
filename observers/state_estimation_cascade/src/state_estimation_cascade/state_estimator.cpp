#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>

#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/conversions/msg_msg.hpp>
#include <tobas_tools/conversions/eigen_msg.hpp>

#include "../../include/state_estimation_cascade/state_estimator.hpp"

#define TIMER_PERIOD 5.
#define IMU_BUF_SIZE 1
#define BAR_BUF_SIZE 500  // 100Hz x 5sec
#define GPS_BUF_SIZE 25   // 5Hz x 5sec
#define VEL_BUF_SIZE 1

using namespace std;
using namespace Eigen;
using namespace dh_std;

StateEstimator::StateEstimator()
  : is_initialized_(false),
    filtered_imu_buf_(IMU_BUF_SIZE),
    bar_buf_(BAR_BUF_SIZE),
    gps_pos_buf_(GPS_BUF_SIZE),
    gps_vel_buf_(VEL_BUF_SIZE),
    yaw_now_(0.),
    yaw_prev_(0.),
    yaw_jump_count_(0)
{
  getRosParams();
  fillUnusedBuffers();
  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&StateEstimator::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);

  check_topics_timer_ =
    nh_.createTimer(ros::Duration(TIMER_PERIOD), &StateEstimator::checkTopicsTimerCb, this);
}

StateEstimator::~StateEstimator()
{
  check_topics_timer_.stop();
}

void StateEstimator::getRosParams()
{
  drone_name_ = dh_ros::getParam<string>("/drone_name");
  gravity_ = dh_ros::getParam<double>("/gravity");
  use_bar_ = dh_ros::getParam("~use_barometer", true);
  use_gps_pos_ = dh_ros::getParam("~use_gps_position", true);
  use_gps_vel_ = dh_ros::getParam("~use_gps_velocity", true);
}
void StateEstimator::fillUnusedBuffers()
{
  constexpr double default_air_pressure = 101325.;  // 海面気圧 [Pa]
  constexpr double default_air_pressure_std = 1.;   // [Pa]
  constexpr double default_hor_pos_std = 3.;        // [m]
  constexpr double default_ver_pos_std = 6.;        // [m]
  constexpr double default_hor_vel_std = 0.1;       // [m/s]
  constexpr double default_ver_vel_std = 0.1;       // [m/s]

  if (!use_bar_)
  {
    BarMsg bar;
    bar.fluid_pressure = default_air_pressure;
    bar.variance = sqr(default_air_pressure_std);
    bar_buf_.fill(bar);
  }

  if (!use_gps_pos_)
  {
    GpsMsg gps;
    gps.latitude = 0.;
    gps.longitude = 0.;
    gps.altitude = 0.;
    gps.position_covariance[0] = sqr(default_hor_pos_std);
    gps.position_covariance[1] = 0.;
    gps.position_covariance[2] = 0.;
    gps.position_covariance[3] = 0.;
    gps.position_covariance[4] = sqr(default_hor_pos_std);
    gps.position_covariance[5] = 0.;
    gps.position_covariance[6] = 0.;
    gps.position_covariance[7] = 0.;
    gps.position_covariance[8] = sqr(default_ver_pos_std);
    gps_pos_buf_.fill(gps);
  }

  if (!use_gps_vel_)
  {
    VelMsg vel;
    vel.vel.vel.vx = 0.;
    vel.vel.vel.vy = 0.;
    vel.vel.vel.vz = 0.;
    vel.vel.covariance[0] = sqr(default_hor_vel_std);
    vel.vel.covariance[1] = 0.;
    vel.vel.covariance[2] = 0.;
    vel.vel.covariance[3] = 0.;
    vel.vel.covariance[4] = sqr(default_hor_vel_std);
    vel.vel.covariance[5] = 0.;
    vel.vel.covariance[6] = 0.;
    vel.vel.covariance[7] = 0.;
    vel.vel.covariance[8] = sqr(default_ver_vel_std);
    gps_vel_buf_.fill(vel);
  }
}

void StateEstimator::registerPublishers()
{
  posevel_pub_ = nh_.advertise<StateMsg>("/" + drone_name_ + "/base_state", 1);
}

void StateEstimator::registerSubscribers()
{
  filtered_imu_sub_ =
    nh_.subscribe("/" + drone_name_ + "/filtered_imu", 1, &StateEstimator::filteredImuCb, this);

  if (use_bar_)
  {
    bar_sub_ =
      nh_.subscribe("/" + drone_name_ + "/air_pressure", 1, &StateEstimator::barometerCb, this);
  }

  if (use_gps_pos_)
  {
    gps_pos_sub_ =
      nh_.subscribe("/" + drone_name_ + "/gps", 1, &StateEstimator::gpsPositionCb, this);
  }

  if (use_gps_vel_)
  {
    gps_vel_sub_ =
      nh_.subscribe("/" + drone_name_ + "/ground_speed", 1, &StateEstimator::gpsVelocityCb, this);
  }
}

bool StateEstimator::isReady()
{
  bool ok = true;

  if (!filtered_imu_buf_.isFull())
  {
    ok = false;
  }

  if (use_bar_ && !bar_buf_.isFull())
  {
    ok = false;
  }

  if (use_gps_pos_ && !gps_pos_buf_.isFull())
  {
    ok = false;
  }

  if (use_gps_vel_ && !gps_vel_buf_.isFull())
  {
    ok = false;
  }

  return ok;
}

void StateEstimator::initialize()
{
  // 静止状態でのセンサデータを平均してゼロ点を決める
  setZeroPositions();

  // 各センサの最新の値を取得する
  auto imu = filtered_imu_buf_.getLatest();
  auto gps = gps_pos_buf_.getLatest();
  auto bar = bar_buf_.getLatest();
  auto vel = gps_vel_buf_.getLatest();

  auto gps_cov = gps.position_covariance;

  double dummy, z_var;
  airPressureToAltitude(bar.fluid_pressure, bar.variance, dummy, z_var);

  cart_filter_.initialize(
    Vector3d::Zero(),                                          // init pos
    Vector3d::Zero(),                                          // init vel
    Vector3d::Zero(),                                          // init accel without gravity
    Vector3d(0., 0., -gravity_),                               // init gravity
    Vector3d(gps_cov[0], gps_cov[4], z_var).asDiagonal(),      // init position covariance
    Map<Matrix3d>(vel.vel.covariance.data()),                  // init velocity covariance
    Map<Matrix3d>(imu.linear_acceleration_covariance.data()),  // init acc covariance
    dh_ros::getParam<int>("~gravity_variance_exp")             // init gravity variance
  );

  t_last_ = ros::Time::now();
}

void StateEstimator::setZeroPositions()
{
  // 緯度，経度[deg]
  double sum_lat = 0.;
  double sum_lon = 0.;
  for (int i = 0; i < GPS_BUF_SIZE; ++i)
  {
    const auto& gps = gps_pos_buf_.get(i);
    sum_lat += gps.latitude;
    sum_lon += gps.longitude;
  }
  lat_0_ = sum_lat / GPS_BUF_SIZE;
  lon_0_ = sum_lon / GPS_BUF_SIZE;

  // 高度[m]
  double sum_pressure = 0.;
  for (int i = 0; i < BAR_BUF_SIZE; ++i)
  {
    const auto& bar = bar_buf_.get(i);
    sum_pressure += bar.fluid_pressure;
  }
  double mean_pressure = sum_pressure / BAR_BUF_SIZE;
  airPressureToAltitude(mean_pressure, alt_0_);
}

void StateEstimator::updatePoseVelMsg()
{
  // 時刻
  state_.header.stamp = ros::Time::now();

  // 位置
  tf::vectorEigenToMsg(cart_filter_.getPosition3D(), state_.pose_vel.pose.position);

  // ロール，ピッチ
  const auto& quat = filtered_imu_buf_.getLatest().orientation;
  auto& rpy = state_.pose_vel.pose.orientation;
  quaternionToEuler(quat.x, quat.y, quat.z, quat.w, rpy.roll, rpy.pitch, yaw_now_);

  // ヨー
  if (yaw_now_ - yaw_prev_ > M_PI)  // 負方向のジャンプを検出
  {
    yaw_jump_count_--;
  }
  else if (yaw_now_ - yaw_prev_ < -M_PI)  // 正方向のジャンプを検出
  {
    yaw_jump_count_++;
  }
  yaw_prev_ = yaw_now_;
  rpy.yaw = (2 * M_PI) * yaw_jump_count_ + yaw_now_;

  // 並進速度
  tf::vectorEigenToMsg(cart_filter_.getVelocity(), state_.pose_vel.twist.linear);

  // 回転速度
  const auto& imu = filtered_imu_buf_.getLatest();
  state_.pose_vel.twist.angular = imu.angular_velocity;
}

void StateEstimator::filteredImuCb(const ImuMsg& imu)
{
  filtered_imu_buf_.add(imu);

  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      initialize();
      is_initialized_ = true;
      dh_ros::rosInfo("State estimator is ready.");
    }
    return;
  }

  // TODO: delayを考慮する
  ros::Time now = ros::Time::now();
  double dt = (now - t_last_).toSec();
  ROS_ASSERT(dt >= 0.);
  t_last_ = now;

  tf::quaternionMsgToEigen(imu.orientation, quat_);
  tf::vectorMsgToEigen(imu.linear_acceleration, a_m_);

  auto imu_copy = imu;
  Matrix3d acc_cov = Map<Matrix3d>(imu_copy.linear_acceleration_covariance.data());

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
  airPressureToAltitude(bar.fluid_pressure, bar.variance, z_abs, z_var);

  double z_m = z_abs - alt_0_;
  cart_filter_.measureAltitude(z_m, z_var);
}

void StateEstimator::gpsPositionCb(const GpsMsg& gps)
{
  gps_pos_buf_.add(gps);

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

  cart_filter_.measurePosition2D(xy_m_, cov);
}

void StateEstimator::gpsVelocityCb(const VelMsg& vel)
{
  gps_vel_buf_.add(vel);

  if (!is_initialized_)
  {
    return;
  }

  tf::linVelMsgToEigen(vel.vel.vel, v_m_);

  boost::array<double, 9> cov_copy = vel.vel.covariance;
  Matrix3d cov = Map<Matrix3d>(cov_copy.data());

  cart_filter_.measureVelocity(v_m_, cov);
}

void StateEstimator::dynamicReconfigureCb(const ConfigType& cfg, uint32_t level)
{
  cart_filter_.reconfigure(cfg.gravity_variance_exp);
}

void StateEstimator::checkTopicsTimerCb(const ros::TimerEvent&)
{
  // IMU
  if (filtered_imu_buf_.isEmpty())
  {
    dh_ros::rosWarn("Filtered IMU data is not received yet.");
  }
  else if (!filtered_imu_buf_.isFull())
  {
    dh_ros::rosInfoOnce("Waiting for Filtered IMU data to be collected.");
  }

  // Barometer
  if (use_bar_)
  {
    if (bar_buf_.isEmpty())
    {
      dh_ros::rosWarn("Barometer data is not received yet.");
    }
    else if (!bar_buf_.isFull())
    {
      dh_ros::rosInfoOnce("Waiting for Barometer data to be collected.");
    }
  }

  // GPS position
  if (use_gps_pos_)
  {
    if (gps_pos_buf_.isEmpty())
    {
      dh_ros::rosWarn("GPS position data is not received yet.");
    }
    else if (!gps_pos_buf_.isFull())
    {
      dh_ros::rosInfoOnce("Waiting for GPS position data to be collected.");
    }
  }

  // GPS velocity
  if (use_gps_vel_)
  {
    if (gps_vel_buf_.isEmpty())
    {
      dh_ros::rosWarn("GPS velocity data is not received yet.");
    }
    else if (!gps_vel_buf_.isFull())
    {
      dh_ros::rosInfoOnce("Waiting for GPS velocity data to be collected.");
    }
  }
}
