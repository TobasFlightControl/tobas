#include <Eigen/Eigen>

#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/boost.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_ros_tools/operators.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/util.hpp>
#include <tobas_ros_tools/time.hpp>
#include <tobas_ros_tools/eigen_conversion.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_common_actions/pre_arm_check_server.hpp"
#include "../include/tobas_common_actions/common.hpp"

using namespace std;
using namespace Eigen;
using namespace tobas_std;

namespace tobas_common_actions
{
PreArmCheckServer::PreArmCheckServer(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    is_action_running_(false),
    bar_alt_buf_(kMeasureTime),
    gps_alt_buf_(kMeasureTime),
    as_(nh_, tobas::kPreArmCheckAction, boost::bind(&self::executeCb, this, _1), false)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  registerPublishers();
  registerSubscribers();

  as_.start();
}

void PreArmCheckServer::getRosParams()
{
}

void PreArmCheckServer::registerPublishers()
{
}

void PreArmCheckServer::registerSubscribers()
{
  bat_sub_ = nh_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batCb, this, tcpNoDelay());
  imu_sub_ = nh_.subscribe(tobas::kImuTopic, 1, &self::imuCb, this, tcpNoDelay());
  mag_sub_ = nh_.subscribe(tobas::kMagTopic, 1, &self::magCb, this, tcpNoDelay());
  bar_sub_ = nh_.subscribe(tobas::kAirPressureTopic, 1, &self::barCb, this, tcpNoDelay());
  gps_sub_ = nh_.subscribe(tobas::kGpsTopic, 1, &self::gpsCb, this, tcpNoDelay());
}

bool PreArmCheckServer::isConditionsMet()
{
  if ((ros::Time::now() - t_meas_start_).toSec() > kMeasureTime)
    return false;

  if (bat_count_ == 0)
    return false;
  if (imu_count_ == 0)
    return false;
  if (mag_count_ == 0)
    return false;
  if (bar_count_ == 0)
    return false;
  if (gps_count_ == 0)
    return false;

  return true;
}

void PreArmCheckServer::reset()
{
  t_meas_start_ = ros::Time::now();

  bat_count_ = 0;
  imu_count_ = 0;
  mag_count_ = 0;
  bar_count_ = 0;
  gps_count_ = 0;

  bat_sum_ = BatMsg();
  imu_sum_ = ImuMsg();
  mag_sum_ = MagMsg();
  bar_sum_ = BarMsg();
  gps_sum_ = GpsMsg();
}

void PreArmCheckServer::fillResult()
{
  result_.bat_count = bat_count_;
  result_.imu_count = imu_count_;
  result_.mag_count = mag_count_;
  result_.bar_count = bar_count_;
  result_.gps_count = gps_count_;

  const double bat_count = static_cast<double>(bat_count_);
  result_.battery.voltage = bat_sum_.voltage / bat_count;
  result_.battery.current = bat_sum_.current / bat_count;

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

void PreArmCheckServer::batCb(const tobas_msgs::BatteryConstPtr& bat)
{
  if (!is_action_running_)
    return;

  // バッテリー電圧が定格電圧より小さければエラー
  if (bat->voltage < drone_.nominalBatteryVoltage())
  {
    is_action_running_ = false;
    result_.error_code = ResultType::BATTERY_VOLTAGE_ERROR;
    as_.setAborted(result_, "Battery voltage is lower than the nominal voltage.");
    return;
  }

  ++bat_count_;

  bat_sum_.voltage += bat->voltage;
  bat_sum_.current += bat->current;
}

void PreArmCheckServer::imuCb(const ImuMsg::ConstPtr& imu)
{
  if (!is_action_running_)
    return;

  // ジャイロの大きさをチェック
  const auto gyro_norm = tobas_ros::norm(imu->angular_velocity);
  if (gyro_norm > kGyroNormThreshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "Rotational movement of the aircraft is detected. Measuring sensor data again...");
    reset();
    return;
  }

  // 加速度の重力ベクトルに対するの誤差をチェック
  const auto acc_err_x = abs(imu->linear_acceleration.x);
  const auto acc_err_y = abs(imu->linear_acceleration.y);
  const auto acc_err_z = abs(imu->linear_acceleration.z - tobas::kGravity);
  if (tobas_std::max(acc_err_x, acc_err_y, acc_err_z) > kAccelErrorThreshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "Translational movement of the aircraft is detected. Measuring sensor data again...");
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

void PreArmCheckServer::magCb(const MagMsg::ConstPtr& mag)
{
  if (!is_action_running_)
    return;

  ++mag_count_;

  mag_sum_.magnetic_field = mag_sum_.magnetic_field + mag->magnetic_field;
  mag_sum_.magnetic_field_covariance =
    mag_sum_.magnetic_field_covariance + mag->magnetic_field_covariance;
}

void PreArmCheckServer::barCb(const BarMsg::ConstPtr& bar)
{
  if (!is_action_running_)
    return;

  const auto bar_alt = pressureToAltitude(bar->fluid_pressure);
  bar_alt_buf_.add(tobas_ros::chronoFromRosTime(bar->header.stamp), bar_alt);

  // 気圧高度の範囲をチェック
  if (bar_alt_buf_.stddev() > kAirAltStddevThreshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "A drift in barometric altitude is detected. Measuring sensor data again...");
    reset();
    return;
  }

  ++bar_count_;

  bar_sum_.fluid_pressure += bar->fluid_pressure;
  bar_sum_.variance += bar->variance;
}

void PreArmCheckServer::gpsCb(const GpsMsg::ConstPtr& gps)
{
  if (!is_action_running_)
    return;

  gps_alt_buf_.add(tobas_ros::chronoFromRosTime(gps->header.stamp), gps->altitude);

  // 気圧高度の範囲をチェック
  if (gps_alt_buf_.stddev() > kGpsAltStddevThreshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_, "A drift in GPS altitude is detected. Measuring sensor data again...");
    reset();
    return;
  }

  // 共分散をチェック
  tobas_ros::matrix3MsgToEigen(gps->position_covariance, gps_pos_cov_);
  const SelfAdjointEigenSolver<Matrix3d> gps_pos_es(gps_pos_cov_);
  const auto max_eigenvalue = gps_pos_es.eigenvalues().z();
  if (max_eigenvalue > sqr(kGpsPosCovStddevThreshold))
  {
    rosWarnThrottle(
      kWarnPeriod, name_, "The accuracy of the GPS is poor. Measuring sensor data again...");
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

void PreArmCheckServer::executeCb(const GoalType&)
{
  rosInfo(name_, "Action is called. Measuring sensor data for " << kMeasureTime << " seconds.");

  is_action_running_ = true;
  reset();

  // ros::Timerではなくros::Rateで時間管理を行う
  // executeCbを先にreturnしてしまうと，アクションが完了する前に次のコールバックが呼ばれてしまう危険性がある？
  ros::Rate rate(kUpdateRate);

  while (nh_.ok())
  {
    // 外部からの中断命令
    if (as_.isPreemptRequested())
    {
      is_action_running_ = false;
      fillResult();
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return;
    }

    // 他のコールバックによりアクションが止められたら異常終了
    if (!is_action_running_)
      return;

    // 一定時間の異常が起きなければ正常終了
    if (isConditionsMet())
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
