#include <actionlib/client/simple_action_client.h>

#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/boost.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_kdl/conversion/kdl_msg.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_ros_tools/eigen_conversion.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/conversions/msg_msg.hpp>

#include "../include/state_estimation_cascade/state_estimator.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;
using namespace tobas_std;

namespace state_estimation_cascade
{
StateEstimator::StateEstimator(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    check_topics_timer_(nh_, kTimerPeriod, &self::checkTopicsTimerCb, this),
    server_(pnh_)
{
  getRosParams();

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&self::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void StateEstimator::getRosParams()
{
  tobas_ros::getParam(pnh_, "use_gps", use_gps_, kDefaultUseGps);
  tobas_ros::getParam(
    pnh_, "gps_horizontal_position_stddev_threshold", gps_hor_pos_stddev_thr_,
    kDefaultGpsHorPosStddevThreshold, tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh_, "gps_vertical_position_stddev_threshold", gps_ver_pos_stddev_thr_,
    kDefaultGpsVerPosStddevThreshold, tobas_ros::POSITIVE);
}

void StateEstimator::registerPublishers()
{
  odom_pub_ = nh_.advertise<OdomMsg>(tobas::kOdometryTopic, 1);
}

void StateEstimator::registerSubscribers()
{
  super::registerSubscribers();

  filtered_imu_sub_ = nh_.subscribe(kFilteredImuTopic, 1, &self::filteredImuCb, this, tcpNoDelay());
  bar_sub_ = nh_.subscribe(tobas::kAirPressureTopic, 1, &self::barometerCb, this, tcpNoDelay());

  if (use_gps_)
  {
    gps_sub_ = nh_.subscribe(tobas::kGpsTopic, 1, &self::gpsPositionCb, this, tcpNoDelay());
  }
}

bool StateEstimator::isReady()
{
  if (!imu_received_)
    return false;
  if (!bar_received_)
    return false;
  if (use_gps_ && !gps_received_)
    return false;

  return true;
}

void StateEstimator::initialize(const ImuMsg& imu)
{
  // 初期化処理の開始時間
  ros::Time start_time = ros::Time::now();

  // 静止状態でのセンサデータを平均してゼロ点を決める
  const auto result = setZeroPositions();

  // カルマンフィルタを初期化
  // 完全な停止状態で起動するため初期状態の不確かさはかなり小さい想定
  cart_filter_.initialize(
    Vector3d::Zero(),                    // Inittial position
    Vector3d::Zero(),                    // Inittial velocity
    Vector3d::Zero(),                    // Inittial acceleration without gravity
    Vector3d(0., 0., -tobas::kGravity),  // Inittial gravity
    Map<const Matrix3d>(result->gps.position_covariance.data()),  // Initial position cov
    Map<const Matrix3d>(result->gps.velocity_covariance.data()),  // Initial velocity cov
    Matrix3d::Zero(),                                             // Initial acceleration cov
    Matrix3d::Zero()                                              // Initial gravity cov
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

tobas_msgs::StaticStateDeterminationResultConstPtr StateEstimator::setZeroPositions()
{
  actionlib::SimpleActionClient<tobas_msgs::StaticStateDeterminationAction> ac(
    tobas::kStaticStateDeterminationAction);
  rosInfo(
    name_,
    "Waiting for action server '" << tobas::kStaticStateDeterminationAction << "' to start.");
  ac.waitForServer();

  rosInfo(
    name_,
    "Action server '" << tobas::kStaticStateDeterminationAction << "' started, sending goal.");
  tobas_msgs::StaticStateDeterminationGoal goal;
  goal.gps_horizontal_position_stddev_threshold = gps_hor_pos_stddev_thr_;
  goal.gps_vertical_position_stddev_threshold = gps_ver_pos_stddev_thr_;
  ac.sendGoal(goal);

  const bool finished_before_timeout = ac.waitForResult();
  if (!finished_before_timeout)
  {
    ROS_THROW_NAMED(
      name_, "'" << tobas::kStaticStateDeterminationAction << "' did not finish before timeout.");
  }

  const auto result = ac.getResult();
  const auto state = ac.getState();
  if (result->error_code != tobas_msgs::StaticStateDeterminationResult::NO_ERROR)
  {
    ROS_THROW_NAMED(
      name_, "'" << tobas::kStaticStateDeterminationAction
                 << "' finished with error: " << state.getText());
  }

  // 経緯度
  lat_0_ = result->gps.latitude;
  lon_0_ = result->gps.longitude;

  // 高度
  alt_0_ = pressureToAltitude(result->air_pressure.fluid_pressure);

  return result;
}

StateEstimator::OdomMsg::ConstPtr StateEstimator::makeOdometryMsg(const ImuMsg& imu)
{
  const auto odom = boost::make_shared<OdomMsg>();

  // Time stamp
  odom->header.stamp = imu.header.stamp;

  // Position
  odom->pose.pos.data = cart_filter_.getPosition();
  tobas_ros::matrix3EigenToMsg(cart_filter_.getPositionCovariance(), odom->position_covariance);

  // Roll, Pitch
  const auto& q = imu.orientation;
  auto& rpy = odom->pose.euler;
  quaternionToEuler(q.x, q.y, q.z, q.w, rpy.roll, rpy.pitch, yaw_now_);

  // Yaw
  if (yaw_now_ - yaw_prev_ > M_PI)  // 負方向のジャンプを検出
    --yaw_jump_count_;
  else if (yaw_now_ - yaw_prev_ < -M_PI)  // 正方向のジャンプを検出
    ++yaw_jump_count_;
  yaw_prev_ = yaw_now_;
  rpy.yaw = (2 * M_PI) * yaw_jump_count_ + yaw_now_;

  odom->orientation_covariance.fill(nan(tobas::kUnknown));  // TODO: 相補フィルタから推定

  // Linear velocity (Local)
  odom->twist.vel.data = cart_filter_.getVelocity();
  odom->twist.vel = rpy.inverse(odom->twist.vel);  // World -> Local
  const Matrix3d R_W_B = rpy.toRotation().data;
  const Matrix3d vel_cov_B = R_W_B.transpose() * cart_filter_.getVelocityCovariance() * R_W_B;
  tobas_ros::matrix3EigenToMsg(vel_cov_B, odom->linear_velocity_covariance);

  // Angular velocity (Local)
  vectorMsgToKDL(imu.angular_velocity, odom->twist.rot);
  odom->angular_velocity_covariance = imu.angular_velocity_covariance;

  // Linear acceleration (Local)
  vectorMsgToKDL(imu.linear_acceleration, odom->accel.linear);
  odom->accel.linear += rpy.inverse(Vector(0, 0, -tobas::kGravity));  // 重力を除く
  odom->linear_acceleration_covariance = imu.linear_acceleration_covariance;

  // Angular acceleration (Local)
  odom->accel.angular.fill(nan(tobas::kUnknown));
  odom->angular_acceleration_covariance.fill(nan(tobas::kUnknown));

  return odom;
}

void StateEstimator::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      check_topics_timer_.stop();
      break;
    default:
      break;
  }
}

void StateEstimator::filteredImuCb(const ImuMsg::ConstPtr& imu)
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
      initialize(*imu);
      is_initialized_ = true;
      TOBAS_GOOD("State estimator is ready.");
    }
    return;
  }

  // TODO: ESKFのようにdtのチェック
  const double dt = (imu->header.stamp - t_last_).toSec();
  t_last_ = imu->header.stamp;
  if (dt <= 0. || kImuTimeGapThreshold < dt)
  {
    return;
  }

  tobas_ros::quaternionMsgToEigen(imu->orientation, quat_);
  tobas_ros::vectorMsgToEigen(imu->linear_acceleration, a_m_);

  auto acc_cov_data = imu->linear_acceleration_covariance;
  Matrix3d acc_cov = Map<Matrix3d>(acc_cov_data.data());

  // 公称状態を更新
  cart_filter_.predict(quat_, acc_cov, dt);

  // 加速度の観測
  cart_filter_.measureAcceleration(a_m_, acc_cov);

  // 推定状態を発行
  const auto odom = makeOdometryMsg(*imu);
  odom_pub_.publish(odom);
}

void StateEstimator::barometerCb(const BarMsg::ConstPtr& bar)
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
  pressureToAltitude(bar->fluid_pressure, bar->variance, z_abs, z_var);

  const double z_m = z_abs - alt_0_;
  cart_filter_.measureAltitude(z_m, z_var);
}

void StateEstimator::gpsPositionCb(const GpsMsg::ConstPtr& gps)
{
  if (!gps_received_)
  {
    gps_received_ = true;
  }

  if (!is_initialized_)
  {
    return;
  }

  // TODO: 位置と速度を同時にフィードバック
  gpsToCartRelative(gps->latitude, gps->longitude, lat_0_, lon_0_, xy_m_.x(), xy_m_.y());
  Matrix2d pos_cov;
  pos_cov(0, 0) = gps->position_covariance[0];
  pos_cov(0, 1) = gps->position_covariance[1];
  pos_cov(1, 0) = gps->position_covariance[3];
  pos_cov(1, 1) = gps->position_covariance[4];
  cart_filter_.measureXY(xy_m_, pos_cov);

  const Matrix3d vel_cov = Map<const Matrix3d>(gps->velocity_covariance.data());
  cart_filter_.measureVelocity(gps->ground_speed.data, vel_cov);
}

void StateEstimator::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!imu_received_)
    rosInfo(name_, "Waiting for " << ns() << kFilteredImuTopic);

  if (!bar_received_)
    rosInfo(name_, "Waiting for " << ns() << tobas::kAirPressureTopic);

  if (use_gps_ && !gps_received_)
    rosInfo(name_, "Waiting for " << ns() << tobas::kGpsTopic);
}

void StateEstimator::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  cart_filter_.configure(cfg.gravity_variance);
}
}  // namespace state_estimation_cascade
