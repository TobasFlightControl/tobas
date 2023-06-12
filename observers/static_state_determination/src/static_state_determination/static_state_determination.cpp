#include <dh_std_tools/math.hpp>
#include <dh_std_tools/boost.hpp>
#include <dh_ros_tools/operators.hpp>

#include "../../include/static_state_determination/static_state_determination.hpp"
#include "../../include/static_state_determination/common.hpp"

using namespace dh_std;

namespace static_state_determination
{
StaticStateDeterminationServer::StaticStateDeterminationServer()
  : super(),
    as_(nh_, kActionName, boost::bind(&StaticStateDeterminationServer::executeCb, this, _1), true)
{
}

void StaticStateDeterminationServer::getRosParams()
{
}

void StaticStateDeterminationServer::registerPublishers()
{
}

void StaticStateDeterminationServer::registerSubscribers()
{
  imu_sub_ = nh_.subscribe("imu", 1, &StaticStateDeterminationServer::imuCb, this);
  mag_sub_ = nh_.subscribe("magnetic_field", 1, &StaticStateDeterminationServer::magCb, this);
  bar_sub_ = nh_.subscribe("air_pressure", 1, &StaticStateDeterminationServer::barCb, this);
  gps_sub_ = nh_.subscribe("gps", 1, &StaticStateDeterminationServer::gpsCb, this);
  vel_sub_ = nh_.subscribe("ground_speed", 1, &StaticStateDeterminationServer::velCb, this);
}

void StaticStateDeterminationServer::reset()
{
  // 各センサ値や共分散には最初に0がかかるため初期化する必要はない

  // 平均値の計算に用いるサンプル数を初期化
  result_.imu_count = 0;
  result_.mag_count = 0;
  result_.bar_count = 0;
  result_.gps_count = 0;
  result_.vel_count = 0;
}

bool StaticStateDeterminationServer::isValidGoal()
{
  if (goal_->gps_position_stddev_threshold <= 0.)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "Position std. dev must be positive.");
    return false;
  }

  return true;
}

bool StaticStateDeterminationServer::isValidResult()
{
  bool ok = true;

  ok &= result_.imu_count > 0;
  ok &= result_.mag_count > 0;
  ok &= result_.bar_count > 0;
  ok &= result_.gps_count > 0;
  ok &= result_.vel_count > 0;

  ok &= feedback_.gps_x_stddev < goal_->gps_position_stddev_threshold;
  ok &= feedback_.gps_y_stddev < goal_->gps_position_stddev_threshold;

  return ok;
}

void StaticStateDeterminationServer::imuCb(const ImuMsg& imu)
{
  ++result_.imu_count;
  const double n = static_cast<double>(result_.imu_count);

  const auto& prev_gyro = result_.imu.angular_velocity;
  const auto& prev_acc = result_.imu.linear_acceleration;
  result_.imu.angular_velocity = ((n - 1) * prev_gyro + imu.angular_velocity) / n;
  result_.imu.linear_acceleration = ((n - 1) * prev_acc + imu.linear_acceleration) / n;

  const auto& prev_gyro_cov = result_.imu.angular_velocity_covariance;
  const auto& prev_acc_cov = result_.imu.linear_acceleration_covariance;
  result_.imu.angular_velocity_covariance =
    (sqr(n - 1) * prev_gyro_cov + imu.angular_velocity_covariance) / sqr(n);
  result_.imu.linear_acceleration_covariance =
    (sqr(n - 1) * prev_acc_cov + imu.linear_acceleration_covariance) / sqr(n);
}

void StaticStateDeterminationServer::magCb(const MagMsg& mag)
{
  ++result_.mag_count;
  const double n = static_cast<double>(result_.mag_count);

  const auto& prev_mag = result_.magnetic_field.magnetic_field;
  result_.magnetic_field.magnetic_field = ((n - 1) * prev_mag + mag.magnetic_field) / n;

  const auto& prev_cov = result_.magnetic_field.magnetic_field_covariance;
  result_.magnetic_field.magnetic_field_covariance =
    (sqr(n - 1) * prev_cov + mag.magnetic_field_covariance) / sqr(n);
}

void StaticStateDeterminationServer::barCb(const BarMsg& bar)
{
  ++result_.bar_count;
  const double n = static_cast<double>(result_.bar_count);

  const auto& prev_bar = result_.air_pressure.fluid_pressure;
  result_.air_pressure.fluid_pressure = ((n - 1) * prev_bar + bar.fluid_pressure) / n;

  const auto& prev_var = result_.air_pressure.variance;
  result_.air_pressure.fluid_pressure = (sqr(n - 1) * prev_var + bar.variance) / sqr(n);
}

void StaticStateDeterminationServer::gpsCb(const GpsMsg& gps)
{
  ++result_.gps_count;
  const double n = static_cast<double>(result_.gps_count);

  const auto& prev_gps = result_.gps;
  result_.gps.latitude = ((n - 1) * prev_gps.latitude + gps.latitude) / n;
  result_.gps.longitude = ((n - 1) * prev_gps.longitude + gps.longitude) / n;
  result_.gps.altitude = ((n - 1) * prev_gps.altitude + gps.altitude) / n;

  const auto& prev_cov = result_.gps.position_covariance;
  result_.gps.position_covariance = (sqr(n - 1) * prev_cov + gps.position_covariance) / sqr(n);
}

void StaticStateDeterminationServer::velCb(const VelMsg& vel)
{
  ++result_.vel_count;
  const double n = static_cast<double>(result_.vel_count);

  const auto& prev_vel = result_.ground_speed.vel;
  result_.ground_speed.vel = ((n - 1) * prev_vel + vel.vel) / n;

  const auto& prev_cov = result_.ground_speed.covariance;
  result_.ground_speed.covariance = (sqr(n - 1) * prev_cov + vel.covariance) / sqr(n);
}

void StaticStateDeterminationServer::executeCb(const GoalType& goal)
{
  goal_ = goal;

  if (!isValidGoal())
  {
    return;
  }

  reset();

  ros::Time start_time = ros::Time::now();
  ros::Rate rate(kUpdateRate);

  while (ros::ok())
  {
    if (as_.isPreemptRequested())
    {
      result_.error_code = ResultType::UNKNOWN_ERROR;
      as_.setPreempted(result_);
      return;
    }

    // フィードバックを発行
    feedback_.gps_x_stddev = sqrt(result_.gps.position_covariance[0]);
    feedback_.gps_y_stddev = sqrt(result_.gps.position_covariance[4]);
    as_.publishFeedback(feedback_);

    // 条件を満たしていれば終了
    if (isValidResult())
    {
      result_.error_code = ResultType::NO_ERROR;
      as_.setSucceeded(result_);
      return;
    }

    ros::spinOnce();
    rate.sleep();
  }
}
}  // namespace static_state_determination
