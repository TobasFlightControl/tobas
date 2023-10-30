#include <dh_std_tools/math.hpp>
#include <dh_std_tools/boost.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_ros_tools/operators.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/util.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_common_actions/static_state_determination_server.hpp"
#include "../include/tobas_common_actions/common.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_common_actions
{
StaticStateDeterminationServer::StaticStateDeterminationServer(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    is_action_running_(false),
    as_(
      nh_,
      tobas::kStaticStateDeterminationAction,
      boost::bind(&StaticStateDeterminationServer::executeCb, this, _1),
      false)
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
  event_sub_ = nh_.subscribe(
    tobas::kEventTopic, 1, &StaticStateDeterminationServer::eventCb, this, tcpNoDelay());
  imu_sub_ =
    nh_.subscribe(tobas::kImuTopic, 1, &StaticStateDeterminationServer::imuCb, this, tcpNoDelay());
  mag_sub_ =
    nh_.subscribe(tobas::kMagTopic, 1, &StaticStateDeterminationServer::magCb, this, tcpNoDelay());
  bar_sub_ = nh_.subscribe(
    tobas::kAirPressureTopic, 1, &StaticStateDeterminationServer::barCb, this, tcpNoDelay());
  gps_sub_ =
    nh_.subscribe(tobas::kGpsTopic, 1, &StaticStateDeterminationServer::gpsCb, this, tcpNoDelay());
}

void StaticStateDeterminationServer::reset()
{
  imu_count_ = 0;
  mag_count_ = 0;
  bar_count_ = 0;
  gps_count_ = 0;

  imu_sum_ = ImuMsg();
  mag_sum_ = MagMsg();
  bar_sum_ = BarMsg();
  gps_sum_ = GpsMsg();

  pressure_alt_stat_.reset();
}

void StaticStateDeterminationServer::fillResult()
{
  result_.imu_count = imu_count_;
  result_.mag_count = mag_count_;
  result_.bar_count = bar_count_;
  result_.gps_count = gps_count_;

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
  result_.gps.ground_speed = gps_sum_.ground_speed / gps_count;
  result_.gps.position_covariance = gps_sum_.position_covariance / sqr(gps_count);
  result_.gps.velocity_covariance = gps_sum_.velocity_covariance / sqr(gps_count);
}

bool StaticStateDeterminationServer::isGoalValid(const GoalType& goal)
{
  if (
    goal->gps_horizontal_position_stddev_threshold <= 0.
    || goal->gps_vertical_position_stddev_threshold <= 0.)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "Position std. dev must be positive.");
    return false;
  }

  return true;
}

bool StaticStateDeterminationServer::isValidResult(const GoalType& goal)
{
  if (imu_count_ < kMinimumImuCount)
    return false;
  if (mag_count_ < kMinimumImuCount)
    return false;
  if (bar_count_ < kMinimumBarCount)
    return false;
  if (gps_count_ < kMinimumGpsCount)
    return false;

  const auto gps_x_stddev = sqrt(gps_sum_.position_covariance[0] / sqr(gps_count_));
  if (gps_x_stddev > goal->gps_horizontal_position_stddev_threshold)
    return false;

  const auto gps_y_stddev = sqrt(gps_sum_.position_covariance[4] / sqr(gps_count_));
  if (gps_y_stddev > goal->gps_horizontal_position_stddev_threshold)
    return false;

  const auto gps_z_stddev = sqrt(gps_sum_.position_covariance[8] / sqr(gps_count_));
  if (gps_z_stddev > goal->gps_vertical_position_stddev_threshold)
    return false;

  return true;
}

bool StaticStateDeterminationServer::isStatic()
{
  // ジャイロが閾値を超えたらダメ
  if (dh_ros::norm(gyro_) > kStaticGyroThreshold)
  {
    result_.error_code = ResultType::NOT_STATIC;
    as_.setAborted(result_, "Rotation of the aircraft is detected.");
    return false;
  }

  // 気圧高度の分散が閾値を超えたらダメ
  if (bar_count_ > kMinimumBarCount)
  {
    if (pressure_alt_stat_.getVariance() > kStaticAirPressureAltVarThreshold)
    {
      result_.error_code = ResultType::NOT_STATIC;
      as_.setAborted(result_, "A drift in barometric altitude is detected.");
      return false;
    }
  }

  return true;
}

void StaticStateDeterminationServer::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      as_.shutdown();
      break;
    default:
      break;
  }
}

void StaticStateDeterminationServer::imuCb(const ImuMsg::ConstPtr& imu)
{
  if (!is_action_running_)
  {
    return;
  }

  ++imu_count_;

  imu_sum_.angular_velocity = imu_sum_.angular_velocity + imu->angular_velocity;
  imu_sum_.linear_acceleration = imu_sum_.linear_acceleration + imu->linear_acceleration;

  imu_sum_.angular_velocity_covariance =
    imu_sum_.angular_velocity_covariance + imu->angular_velocity_covariance;
  imu_sum_.linear_acceleration_covariance =
    imu_sum_.linear_acceleration_covariance + imu->linear_acceleration_covariance;

  gyro_ = imu->angular_velocity;
}

void StaticStateDeterminationServer::magCb(const MagMsg::ConstPtr& mag)
{
  if (!is_action_running_)
  {
    return;
  }

  ++mag_count_;

  mag_sum_.magnetic_field = mag_sum_.magnetic_field + mag->magnetic_field;
  mag_sum_.magnetic_field_covariance =
    mag_sum_.magnetic_field_covariance + mag->magnetic_field_covariance;
}

void StaticStateDeterminationServer::barCb(const BarMsg::ConstPtr& bar)
{
  if (!is_action_running_)
  {
    return;
  }

  ++bar_count_;

  bar_sum_.fluid_pressure += bar->fluid_pressure;
  bar_sum_.variance += bar->variance;

  const auto pressure_alt = pressureToAltitude(bar->fluid_pressure);
  pressure_alt_stat_.addData(pressure_alt);
}

void StaticStateDeterminationServer::gpsCb(const GpsMsg::ConstPtr& gps)
{
  if (!is_action_running_)
  {
    return;
  }

  ++gps_count_;

  gps_sum_.latitude += gps->latitude;
  gps_sum_.longitude += gps->longitude;
  gps_sum_.altitude += gps->altitude;
  gps_sum_.position_covariance = gps_sum_.position_covariance + gps->position_covariance;

  gps_sum_.ground_speed += gps->ground_speed;
  gps_sum_.velocity_covariance = gps_sum_.velocity_covariance + gps->velocity_covariance;
}

void StaticStateDeterminationServer::executeCb(const GoalType& goal)
{
  rosInfo(name_, "Action is called.");

  if (!isGoalValid(goal))
  {
    return;
  }

  reset();
  is_action_running_ = true;

  ros::Rate rate(kUpdateRate);
  while (nh_.ok())
  {
    if (as_.isPreemptRequested())
    {
      is_action_running_ = false;
      fillResult();
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return;
    }

    // 静止していなければ終了
    if (!isStatic())
    {
      return;
    }

    if (gps_count_ > 0)
    {
      // フィードバックを発行
      feedback_.gps_x_stddev = sqrt(gps_sum_.position_covariance[0] / sqr(gps_count_));
      feedback_.gps_y_stddev = sqrt(gps_sum_.position_covariance[4] / sqr(gps_count_));
      feedback_.gps_z_stddev = sqrt(gps_sum_.position_covariance[8] / sqr(gps_count_));
      as_.publishFeedback(feedback_);

      // コンソールにもフィードバックを出す
      rosInfoThrottle(
        kInfoPeriod, name_,
        "GPS position std. dev [m]: (" << feedback_.gps_x_stddev << ", " << feedback_.gps_y_stddev
                                       << ", " << feedback_.gps_z_stddev << ")");
    }

    // 条件を満たしていれば終了
    if (isValidResult(goal))
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
}
}  // namespace tobas_common_actions
