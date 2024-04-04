#include <tobas_std_tools/math.hpp>
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
  : super(nh, pnh, name)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  registerPublishers();
  registerSubscribers();

  ss_ = nh_.advertiseService(tobas::kPreArmCheckSrv, &self::executeCb, this);
}

void PreArmCheckServer::getRosParams()
{
}

void PreArmCheckServer::registerPublishers()
{
}

void PreArmCheckServer::registerSubscribers()
{
  arming_sub_ = nh_.subscribe(tobas::kArmingTopic, 1, &self::armingCb, this);
  battery_sub_ = nh_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this);
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this);
}

void PreArmCheckServer::reset()
{
  battery_ = nullptr;
  odom_ = nullptr;
}

void PreArmCheckServer::armingCb(const std_msgs::BoolConstPtr& arming)
{
  if (arming->data != arming_->data)
    reset();

  arming_ = arming;
}

void PreArmCheckServer::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  if (arming_->data)
    return;

  battery_ = battery;
}

void PreArmCheckServer::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  if (arming_->data)
    return;

  odom_ = odom;
}

bool PreArmCheckServer::executeCb(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res)
{
  res.success = false;

  // 各メッセージが正しく流れていることを確認
  if (battery_ == nullptr)
  {
    res.message = "Battery message is not received yet.";
    return true;
  }
  if (odom_ == nullptr)
  {
    res.message = "Odometry message is not received yet.";
    return true;
  }

  // バッテリー電圧が定格電圧以上であることを確認
  if (battery_->voltage < drone_.batteryConfig().nominal_voltage)
  {
    res.message = "Battery voltage is lower than the nominal voltage.";
    return true;
  }

  // ジャイロの大きさが閾値以下であることを確認
  // TODO: 一定時間満たしていることを確認
  const auto gyro_norm = odom_->twist.rot.norm();
  if (gyro_norm > kGyroNormThreshold)
  {
    res.message = "Rotational movement of the aircraft is detected.";
    return true;
  }

  // 状態推定が良好であることを確認
  if (odom_->status != tobas_msgs::Odometry::NO_ERROR)
  {
    res.message = "There is an anomaly in the state estimation.";
    return true;
  }

  // 位置推定の共分散が閾値以下であることを確認
  tobas_ros::matrix3MsgToEigen(odom_->position_covariance, cov_);
  const auto hor_pos_var = max(cov_(0, 0), cov_(1, 1));
  const auto ver_pos_var = cov_(2, 2);
  if (hor_pos_var > sqr(kHorPosStddevThreshold))
  {
    res.message = "The accuracy of horizontal position estimation is too low.";
    return true;
  }
  if (ver_pos_var > sqr(kVerPosStddevThreshold))
  {
    res.message = "The accuracy of vertical position estimation is too low.";
    return true;
  }

  // 姿勢推定の共分散が閾値以下であることを確認
  tobas_ros::matrix3MsgToEigen(odom_->orientation_covariance, cov_);
  const auto rot_var = cov_.diagonal().maxCoeff();
  if (rot_var > sqr(kRotStddevThreshold))
  {
    res.message = "The accuracy of orientation estimation is too low.";
    return true;
  }

  // 速度推定の共分散が閾値以下であることを確認
  tobas_ros::matrix3MsgToEigen(odom_->linear_velocity_covariance, cov_);
  const auto vel_var = cov_.diagonal().maxCoeff();
  if (vel_var > sqr(kVelStddevThreshold))
  {
    res.message = "The accuracy of linear velocity estimation is too low.";
    return true;
  }

  res.success = true;
  return true;
}
}  // namespace tobas_common_actions
