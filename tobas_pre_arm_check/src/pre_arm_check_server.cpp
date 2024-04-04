#include <tobas_std_tools/math.hpp>
#include <tobas_ros_tools/eigen_conversion.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_pre_arm_check/pre_arm_check_server.hpp"
#include "../include/tobas_pre_arm_check/common.hpp"

using namespace std;
using namespace Eigen;
using namespace tobas_std;

namespace tobas_pre_arm_check
{
PreArmCheckServer::PreArmCheckServer(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  pre_arm_check_.error_code = tobas_msgs::PreArmCheck::E_NOT_CONDUCTED;

  registerPublishers();
  registerSubscribers();
  pre_arm_check_ss_ = nh_.advertiseService(tobas::kPreArmCheckSrv, &self::preArmCheckSrvCb, this);
  pre_arm_check_timer_ = nh_.createTimer(kPreArmCheckTimerRate, &self::preArmCheckTimerCb, this);
}

void PreArmCheckServer::getRosParams()
{
}

void PreArmCheckServer::registerPublishers()
{
  pre_arm_check_pub_ = nh_.advertise<tobas_msgs::PreArmCheck>(tobas::kPreArmCheckTopic, 1, true);
}

void PreArmCheckServer::registerSubscribers()
{
  arming_sub_ = nh_.subscribe(tobas::kArmingTopic, 1, &self::armingCb, this);
  battery_sub_ = nh_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this);
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this);
}

void PreArmCheckServer::armingCb(const std_msgs::BoolConstPtr& arming)
{
  arming_ = arming;
}

void PreArmCheckServer::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void PreArmCheckServer::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  odom_ = odom;
}

bool PreArmCheckServer::preArmCheckSrvCb(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res)
{
  res.success = pre_arm_check_.error_code >= 0;

  switch (pre_arm_check_.error_code)
  {
    case tobas_msgs::PreArmCheck::E_ALREADY_ARMED:
      res.message = "The rotors are already armed.";
      break;
    case tobas_msgs::PreArmCheck::E_NO_ERROR:
      res.message = "No error.";
      break;
    case tobas_msgs::PreArmCheck::E_NOT_CONDUCTED:
      res.message = "Pre-arm check is not conducted yet.";
      break;
    case tobas_msgs::PreArmCheck::E_BATTERY_NOT_RECEIVED:
      res.message = "Battery message is not received yet.";
      break;
    case tobas_msgs::PreArmCheck::E_ODOMETRY_NOT_RECEIVED:
      res.message = "Odometry message is not received yet.";
      break;
    case tobas_msgs::PreArmCheck::E_BATTERY_VOLTAGE_TOO_LOW:
      res.message = "Battery voltage is lower than the nominal voltage.";
      break;
    case tobas_msgs::PreArmCheck::E_GYRO_TOO_LARGE:
      res.message = "Rotational movement of the aircraft is detected.";
      break;
    case tobas_msgs::PreArmCheck::E_STATE_ESTIMATION_ISSUE:
      res.message = "There is an anomaly in the state estimation.";
      break;
    case tobas_msgs::PreArmCheck::E_HORIZONTAL_POSITION_ACCURACY_POOR:
      res.message = "The accuracy of horizontal position estimation is too low.";
      break;
    case tobas_msgs::PreArmCheck::E_VERTICAL_POSITION_ACCURACY_POOR:
      res.message = "The accuracy of vertical position estimation is too low.";
      break;
    case tobas_msgs::PreArmCheck::E_ORIENTATION_ACCURACY_POOR:
      res.message = "The accuracy of orientation estimation is too low.";
      break;
    case tobas_msgs::PreArmCheck::E_LINEAR_VELOCITY_ACCURACY_POOR:
      res.message = "The accuracy of linear velocity estimation is too low.";
      break;
    default:
      res.message = "Unknown error code.";
      break;
  }

  return true;
}

void PreArmCheckServer::preArmCheckTimerCb(const ros::TimerEvent& event)
{
  pre_arm_check_.header.stamp = event.current_real;

  // 既にアームされている場合
  if (arming_->data)
  {
    pre_arm_check_.error_code = tobas_msgs::PreArmCheck::E_ALREADY_ARMED;
    pre_arm_check_pub_.publish(pre_arm_check_);
    return;
  }

  // 各メッセージが正しく流れていることを確認
  if (battery_ == nullptr)
  {
    pre_arm_check_.error_code = tobas_msgs::PreArmCheck::E_BATTERY_NOT_RECEIVED;
    pre_arm_check_pub_.publish(pre_arm_check_);
    return;
  }
  if (odom_ == nullptr)
  {
    pre_arm_check_.error_code = tobas_msgs::PreArmCheck::E_ODOMETRY_NOT_RECEIVED;
    pre_arm_check_pub_.publish(pre_arm_check_);
    return;
  }

  // バッテリー電圧が定格電圧以上であることを確認
  if (battery_->voltage < drone_.batteryConfig().nominal_voltage)
  {
    pre_arm_check_.error_code = tobas_msgs::PreArmCheck::E_BATTERY_VOLTAGE_TOO_LOW;
    pre_arm_check_pub_.publish(pre_arm_check_);
    return;
  }

  // ジャイロの大きさが閾値以下であることを確認
  // TODO: 一定時間満たしていることを確認
  const auto gyro_norm = odom_->twist.rot.norm();
  if (gyro_norm > kGyroNormThreshold)
  {
    pre_arm_check_.error_code = tobas_msgs::PreArmCheck::E_GYRO_TOO_LARGE;
    pre_arm_check_pub_.publish(pre_arm_check_);
    return;
  }

  // 状態推定が良好であることを確認
  if (odom_->status != tobas_msgs::Odometry::NO_ERROR)
  {
    pre_arm_check_.error_code = tobas_msgs::PreArmCheck::E_STATE_ESTIMATION_ISSUE;
    pre_arm_check_pub_.publish(pre_arm_check_);
    return;
  }

  // 位置推定の共分散が閾値以下であることを確認
  tobas_ros::matrix3MsgToEigen(odom_->position_covariance, cov_);
  const auto hor_pos_var = max(cov_(0, 0), cov_(1, 1));
  const auto ver_pos_var = cov_(2, 2);
  if (hor_pos_var > sqr(kHorPosStddevThreshold))
  {
    pre_arm_check_.error_code = tobas_msgs::PreArmCheck::E_HORIZONTAL_POSITION_ACCURACY_POOR;
    pre_arm_check_pub_.publish(pre_arm_check_);
    return;
  }
  if (ver_pos_var > sqr(kVerPosStddevThreshold))
  {
    pre_arm_check_.error_code = tobas_msgs::PreArmCheck::E_VERTICAL_POSITION_ACCURACY_POOR;
    pre_arm_check_pub_.publish(pre_arm_check_);
    return;
  }

  // 姿勢推定の共分散が閾値以下であることを確認
  tobas_ros::matrix3MsgToEigen(odom_->orientation_covariance, cov_);
  const auto rot_var = cov_.diagonal().maxCoeff();
  if (rot_var > sqr(kRotStddevThreshold))
  {
    pre_arm_check_.error_code = tobas_msgs::PreArmCheck::E_ORIENTATION_ACCURACY_POOR;
    pre_arm_check_pub_.publish(pre_arm_check_);
    return;
  }

  // 速度推定の共分散が閾値以下であることを確認
  tobas_ros::matrix3MsgToEigen(odom_->linear_velocity_covariance, cov_);
  const auto vel_var = cov_.diagonal().maxCoeff();
  if (vel_var > sqr(kVelStddevThreshold))
  {
    pre_arm_check_.error_code = tobas_msgs::PreArmCheck::E_LINEAR_VELOCITY_ACCURACY_POOR;
    pre_arm_check_pub_.publish(pre_arm_check_);
    return;
  }

  pre_arm_check_.error_code = tobas_msgs::PreArmCheck::E_NO_ERROR;
  pre_arm_check_pub_.publish(pre_arm_check_);
}
}  // namespace tobas_pre_arm_check
