#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/boost.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_ros_tools/operators.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/util.hpp>
#include <tobas_ros_tools/time.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_common_actions/static_state_determination_server.hpp"
#include "../include/tobas_common_actions/common.hpp"

using namespace std;
using namespace tobas_std;

namespace tobas_common_actions
{
StaticStateDeterminationServer::StaticStateDeterminationServer(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    is_action_running_(false),
    bar_alt_buf_(kMeasureTime),
    gps_alt_buf_(kMeasureTime),
    as_(nh_, tobas::kStaticStateDeterminationAction, boost::bind(&self::executeCb, this, _1), false)
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
  imu_sub_ = nh_.subscribe(tobas::kImuTopic, 1, &self::imuCb, this, tcpNoDelay());
  mag_sub_ = nh_.subscribe(tobas::kMagTopic, 1, &self::magCb, this, tcpNoDelay());
  bar_sub_ = nh_.subscribe(tobas::kAirPressureTopic, 1, &self::barCb, this, tcpNoDelay());
  gps_sub_ = nh_.subscribe(tobas::kGpsTopic, 1, &self::gpsCb, this, tcpNoDelay());
}

void StaticStateDeterminationServer::reset()
{
  t_meas_start_ = ros::Time::now();

  imu_count_ = 0;
  mag_count_ = 0;
  bar_count_ = 0;
  gps_count_ = 0;

  imu_sum_ = ImuMsg();
  mag_sum_ = MagMsg();
  bar_sum_ = BarMsg();
  gps_sum_ = GpsMsg();
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

void StaticStateDeterminationServer::imuCb(const ImuMsg::ConstPtr& imu)
{
  if (!is_action_running_)
    return;

  // ジャイロの大きさをチェック
  if (tobas_ros::norm(imu->angular_velocity) > kGyroThreshold)
  {
    rosWarn(name_, "Rotation of the aircraft is detected. Measuring sensor data again...");
    reset();
    return;
  }

  ++imu_count_;

  imu_sum_.angular_velocity = imu_sum_.angular_velocity + imu->angular_velocity;
  imu_sum_.linear_acceleration = imu_sum_.linear_acceleration + imu->linear_acceleration;

  imu_sum_.angular_velocity_covariance =
    imu_sum_.angular_velocity_covariance + imu->angular_velocity_covariance;
  imu_sum_.linear_acceleration_covariance =
    imu_sum_.linear_acceleration_covariance + imu->linear_acceleration_covariance;
}

void StaticStateDeterminationServer::magCb(const MagMsg::ConstPtr& mag)
{
  if (!is_action_running_)
    return;

  ++mag_count_;

  mag_sum_.magnetic_field = mag_sum_.magnetic_field + mag->magnetic_field;
  mag_sum_.magnetic_field_covariance =
    mag_sum_.magnetic_field_covariance + mag->magnetic_field_covariance;
}

void StaticStateDeterminationServer::barCb(const BarMsg::ConstPtr& bar)
{
  if (!is_action_running_)
    return;

  const auto bar_alt = pressureToAltitude(bar->fluid_pressure);
  bar_alt_buf_.add(tobas_ros::chronoFromRosTime(bar->header.stamp), bar_alt);

  // 気圧高度の範囲をチェック
  if (bar_alt_buf_.stddev() > kAirAltStddevThreshold)
  {
    rosWarn(name_, "A drift in barometric altitude is detected. Measuring sensor data again...");
    reset();
    return;
  }

  ++bar_count_;

  bar_sum_.fluid_pressure += bar->fluid_pressure;
  bar_sum_.variance += bar->variance;
}

void StaticStateDeterminationServer::gpsCb(const GpsMsg::ConstPtr& gps)
{
  if (!is_action_running_)
    return;

  gps_alt_buf_.add(tobas_ros::chronoFromRosTime(gps->header.stamp), gps->altitude);

  // 気圧高度の範囲をチェック
  if (gps_alt_buf_.stddev() > kGpsAltStddevThreshold)
  {
    rosWarn(name_, "A drift in GPS altitude is detected. Measuring sensor data again...");
    reset();
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

void StaticStateDeterminationServer::executeCb(const GoalType&)
{
  rosInfo(name_, "Action is called. Measuring sensor data for " << kMeasureTime << " seconds.");

  is_action_running_ = true;
  reset();

  // ros::Timerではなくros::Rateで時間管理を行う
  // executeCbを先にreturnしてしまうと，アクションが完了する前に次のコールバックが呼ばれてしまう危険性がある？
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

    // 何事もなく測定時間が経過したら終了
    if ((ros::Time::now() - t_meas_start_).toSec() > kMeasureTime)
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
