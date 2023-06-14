#include <dh_std_tools/math.hpp>
#include <dh_std_tools/boost.hpp>
#include <dh_ros_tools/operators.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/static_state_determination/static_state_determination.hpp"
#include "../../include/static_state_determination/common.hpp"

using namespace dh_std;

namespace static_state_determination
{
StaticStateDeterminationServer::StaticStateDeterminationServer()
  : super(),
    is_action_running_(false),
    as_(nh_, kActionName, boost::bind(&StaticStateDeterminationServer::executeCb, this, _1), false)
{
  getRosParams();
  registerPublishers();
  registerSubscribers();

  as_.start();
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
  imu_count_ = 0;
  mag_count_ = 0;
  bar_count_ = 0;
  gps_count_ = 0;
  vel_count_ = 0;

  imu_sum_ = ImuMsg();
  mag_sum_ = MagMsg();
  bar_sum_ = BarMsg();
  gps_sum_ = GpsMsg();
  vel_sum_ = VelMsg();
}

void StaticStateDeterminationServer::fillResult()
{
  result_.imu_count = imu_count_;
  result_.mag_count = mag_count_;
  result_.bar_count = bar_count_;
  result_.gps_count = gps_count_;
  result_.vel_count = vel_count_;

  const double imu_count = static_cast<double>(imu_count_);
  result_.imu.angular_velocity = imu_sum_.angular_velocity / imu_count;
  result_.imu.linear_acceleration = imu_sum_.linear_acceleration / imu_count;
  result_.imu.angular_velocity_covariance = imu_sum_.angular_velocity_covariance / sqr(imu_count);
  result_.imu.linear_acceleration_covariance =
    imu_sum_.linear_acceleration_covariance / sqr(imu_count);

  const double mag_count = static_cast<double>(mag_count_);
  result_.magnetic_field.magnetic_field = mag_sum_.magnetic_field / mag_count;
  result_.magnetic_field.magnetic_field_covariance =
    mag_sum_.magnetic_field_covariance / sqr(mag_count);

  const double bar_count = static_cast<double>(bar_count_);
  result_.air_pressure.fluid_pressure = bar_sum_.fluid_pressure / bar_count;
  result_.air_pressure.variance = bar_sum_.variance / sqr(bar_count);

  const double gps_count = static_cast<double>(gps_count_);
  result_.gps.latitude = gps_sum_.latitude / gps_count;
  result_.gps.longitude = gps_sum_.longitude / gps_count;
  result_.gps.altitude = gps_sum_.altitude / gps_count;
  result_.gps.position_covariance = gps_sum_.position_covariance / sqr(gps_count);

  const double vel_count = static_cast<double>(vel_count_);
  result_.ground_speed.vel = vel_sum_.vel / vel_count;
  result_.ground_speed.covariance = vel_sum_.covariance / sqr(vel_count);
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
  if (imu_count_ == 0)
    return false;
  if (mag_count_ == 0)
    return false;
  if (bar_count_ == 0)
    return false;
  if (gps_count_ == 0)
    return false;
  if (vel_count_ == 0)
    return false;

  const auto gps_x_stddev = sqrt(gps_sum_.position_covariance[0] / sqr(gps_count_));
  if (gps_x_stddev > goal_->gps_position_stddev_threshold)
    return false;

  const auto gps_y_stddev = sqrt(gps_sum_.position_covariance[4] / sqr(gps_count_));
  if (gps_y_stddev > goal_->gps_position_stddev_threshold)
    return false;

  return true;
}

void StaticStateDeterminationServer::imuCb(const ImuMsg& imu)
{
  if (!is_action_running_)
  {
    return;
  }

  ++imu_count_;

  imu_sum_.angular_velocity = imu_sum_.angular_velocity + imu.angular_velocity;
  imu_sum_.linear_acceleration = imu_sum_.linear_acceleration + imu.linear_acceleration;

  imu_sum_.angular_velocity_covariance =
    imu_sum_.angular_velocity_covariance + imu.angular_velocity_covariance;
  imu_sum_.linear_acceleration_covariance =
    imu_sum_.linear_acceleration_covariance + imu.linear_acceleration_covariance;
}

void StaticStateDeterminationServer::magCb(const MagMsg& mag)
{
  if (!is_action_running_)
  {
    return;
  }

  ++mag_count_;

  mag_sum_.magnetic_field = mag_sum_.magnetic_field + mag.magnetic_field;
  mag_sum_.magnetic_field_covariance =
    mag_sum_.magnetic_field_covariance + mag.magnetic_field_covariance;
}

void StaticStateDeterminationServer::barCb(const BarMsg& bar)
{
  if (!is_action_running_)
  {
    return;
  }

  ++bar_count_;

  bar_sum_.fluid_pressure += bar.fluid_pressure;
  bar_sum_.variance += bar.variance;
}

void StaticStateDeterminationServer::gpsCb(const GpsMsg& gps)
{
  if (!is_action_running_)
  {
    return;
  }

  ++gps_count_;

  // FIXME: 数値誤差を発生させないように和をとる．少数部分だけ計算するとか．
  gps_sum_.latitude += gps.latitude;
  gps_sum_.longitude += gps.longitude;
  gps_sum_.altitude += gps.altitude;

  gps_sum_.position_covariance = gps_sum_.position_covariance + gps.position_covariance;
}

void StaticStateDeterminationServer::velCb(const VelMsg& vel)
{
  if (!is_action_running_)
  {
    return;
  }

  ++vel_count_;

  vel_sum_.vel += vel.vel;
  vel_sum_.covariance = vel_sum_.covariance + vel.covariance;
}

void StaticStateDeterminationServer::executeCb(const GoalType& goal)
{
  goal_ = goal;

  if (!isValidGoal())
  {
    return;
  }

  reset();
  is_action_running_ = true;

  ros::Time start_time = ros::Time::now();
  ros::Rate rate(kUpdateRate);

  while (ros::ok())
  {
    if (as_.isPreemptRequested())
    {
      is_action_running_ = false;
      fillResult();
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return;
    }

    if (gps_count_ > 0)
    {
      // フィードバックを発行
      feedback_.gps_x_stddev = sqrt(gps_sum_.position_covariance[0] / sqr(gps_count_));
      feedback_.gps_y_stddev = sqrt(gps_sum_.position_covariance[4] / sqr(gps_count_));
      as_.publishFeedback(feedback_);

      // コンソールにもフィードバックを出す
      rosInfoThrottle(kInfoPeriod, "X std. dev: " << feedback_.gps_x_stddev << "[m]");
      rosInfoThrottle(kInfoPeriod, "Y std. dev: " << feedback_.gps_y_stddev << "[m]");
    }

    // 条件を満たしていれば終了
    if (isValidResult())
    {
      is_action_running_ = false;
      fillResult();
      result_.error_code = ResultType::NO_ERROR;
      as_.setSucceeded(result_);
      return;
    }

    ros::spinOnce();
    rate.sleep();
  }

  is_action_running_ = false;
}
}  // namespace static_state_determination
