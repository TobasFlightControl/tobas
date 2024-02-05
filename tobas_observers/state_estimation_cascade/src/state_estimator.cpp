#include <actionlib/client/simple_action_client.h>

#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/boost.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_ros_tools/eigen_conversion.hpp>
#include <tobas_kdl_msgs/conversion/kdl_msg.hpp>

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

  // Fill the static part of the transform message
  tf_.header.frame_id = tobas::kWorldFrame;
  tf_.child_frame_id = drone_.tree().getRootName();

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&self::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void StateEstimator::getRosParams()
{
  tobas_ros::getParam(pnh_, "use_gps", use_gps_, kDefaultUseGps);
}

void StateEstimator::registerPublishers()
{
  odom_pub_ = nh_.advertise<OdomMsg>(tobas::kOdometryTopic, 1);
}

void StateEstimator::registerSubscribers()
{
  filtered_imu_sub_ = nh_.subscribe(kFilteredImuTopic, 1, &self::filteredImuCb, this, tcpNoDelay());
  bar_sub_ = nh_.subscribe(tobas::kAirPressureTopic, 1, &self::barometerCb, this, tcpNoDelay());

  if (use_gps_)
    gps_sub_ = nh_.subscribe(tobas::kGpsTopic, 1, &self::gpsPositionCb, this, tcpNoDelay());
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

  // IMUのタイムスタンプに初期化に要した時間を足した時間を最新のセンサ時間とする
  const auto duration = ros::Time::now() - start_time;
  t_last_ = imu.header.stamp + duration;
}

tobas_msgs::PreArmCheckResultConstPtr StateEstimator::setZeroPositions()
{
  actionlib::SimpleActionClient<tobas_msgs::PreArmCheckAction> ac(tobas::kPreArmCheckAction);
  rosInfo(name_, "Waiting for action server '" << tobas::kPreArmCheckAction << "' to start.");
  ac.waitForServer();

  rosInfo(name_, "Action server '" << tobas::kPreArmCheckAction << "' started, sending goal.");
  tobas_msgs::PreArmCheckGoal goal;
  ac.sendGoal(goal);

  const bool finished_before_timeout = ac.waitForResult();
  if (!finished_before_timeout)
  {
    ROS_THROW_NAMED(name_, "'" << tobas::kPreArmCheckAction << "' did not finish before timeout.");
  }

  const auto result = ac.getResult();
  const auto state = ac.getState();
  if (result->error_code != tobas_msgs::PreArmCheckResult::NO_ERROR)
  {
    ROS_THROW_NAMED(
      name_, "'" << tobas::kPreArmCheckAction << "' finished with error: " << state.getText());
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

  // Header
  odom->header.stamp = imu.header.stamp;
  odom->header.frame_id = tobas::kWorldFrame;

  // Position (Global)
  odom->frame.p.data = cart_filter_.getPosition();
  tobas_ros::matrix3EigenToMsg(cart_filter_.getPositionCovariance(), odom->position_covariance);

  // Orientation (Global)
  rotationMsgToKDL(imu.orientation, odom->frame.M);
  odom->orientation_covariance.fill(nan(tobas::kUnknown));  // TODO: 相補フィルタから推定

  // Linear velocity (Local)
  odom->twist.vel.data = cart_filter_.getVelocity();
  odom->twist.vel = odom->frame.M.inverse(odom->twist.vel);  // World -> Local
  const auto& R_W_B = odom->frame.M.data;
  const Matrix3d vel_cov_B = R_W_B.transpose() * cart_filter_.getVelocityCovariance() * R_W_B;
  tobas_ros::matrix3EigenToMsg(vel_cov_B, odom->linear_velocity_covariance);

  // Angular velocity (Local)
  vectorMsgToKDL(imu.angular_velocity, odom->twist.rot);
  odom->angular_velocity_covariance = imu.angular_velocity_covariance;

  // Linear acceleration (Local)
  vectorMsgToKDL(imu.linear_acceleration, odom->accel.linear);
  odom->accel.linear += odom->frame.M.inverse(Vector(0, 0, -tobas::kGravity));  // 重力を除く
  odom->linear_acceleration_covariance = imu.linear_acceleration_covariance;

  // Angular acceleration (Local)
  odom->accel.angular.fill(nan(tobas::kUnknown));
  odom->angular_acceleration_covariance.fill(nan(tobas::kUnknown));

  return odom;
}

void StateEstimator::filteredImuCb(const ImuMsg::ConstPtr& imu)
{
  if (!imu_received_)
    imu_received_ = true;

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
  if (dt <= 0 || kImuTimeGapThreshold < dt)
    return;

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

  // TFを発行
  tf_.header.stamp = odom->header.stamp;
  transformKDLToMsg(odom->frame, tf_.transform);
  tf_br_.sendTransform(tf_);
}

void StateEstimator::barometerCb(const BarMsg::ConstPtr& bar)
{
  if (!bar_received_)
    bar_received_ = true;

  if (!is_initialized_)
    return;

  double z_abs, z_var;
  pressureToAltitude(bar->fluid_pressure, bar->variance, z_abs, z_var);

  const double z_m = z_abs - alt_0_;
  cart_filter_.measureAltitude(z_m, z_var);
}

void StateEstimator::gpsPositionCb(const GpsMsg::ConstPtr& gps)
{
  if (!gps_received_)
    gps_received_ = true;

  if (!is_initialized_)
    return;

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
