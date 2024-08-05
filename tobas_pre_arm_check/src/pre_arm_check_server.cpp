#include <tobas_math/core.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_ros2_tools/eigen_conversion.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_pre_arm_check/pre_arm_check_server.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_pre_arm_check
{
PreArmCheckServer::PreArmCheckServer(, const string& name)
  : super(node, pnh, name),
    pos_buf_{ tobas_std::TimestampedBufferDouble(kPosDriftCheckTimeWindow),
              tobas_std::TimestampedBufferDouble(kPosDriftCheckTimeWindow),
              tobas_std::TimestampedBufferDouble(kPosDriftCheckTimeWindow) }
{
  drone_.loadFromParam(node_);

  pre_arm_check_pub_ = node_.advertise<tobas_msgs::PreArmCheck>(tobas::kPreArmCheckTopic, 1, true);

  battery_sub_ = node_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this);
  odom_sub_ = node_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this);

  pre_arm_check_ss_ = node_.advertiseService(tobas::kPreArmCheckSrv, &self::preArmCheckSrvCb, this);

  pre_arm_check_timer_ = node_.createTimer(kPreArmCheckTimerRate, &self::preArmCheckTimerCb, this);
}

void PreArmCheckServer::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void PreArmCheckServer::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  if (odom_ == nullptr)
  {
    odom_ = odom;
    return;
  }

  // 評価時の計算量を抑えるために処理頻度を制限
  if ((odom->header.stamp - odom_->header.stamp).seconds() < kOdomCallbackInterval)
    return;

  odom_ = odom;

  const auto stamp = ros2::chronoFromRosTime(odom->header.stamp);
  for (size_t i = 0; i < 3; ++i)
    pos_buf_[i].add(stamp, odom->frame.p(i));
}

bool PreArmCheckServer::preArmCheckSrvCb(std_srvs::srv::Trigger::Request&, std_srvs::srv::Trigger::Response& res)
{
  // 問題がなければ終了
  res.success = pre_arm_check_.ok;
  if (res.success)
    return true;

  // 問題があるなら1番始めのメッセージを返す
  if (!pre_arm_check_.battery_voltage_sufficient)
  {
    res.message = "Battery voltage is too low.";
    return true;
  }
  if (!pre_arm_check_.attitude_horizontal)
  {
    res.message = "Attitude angle is too large.";
    return true;
  }
  if (!pre_arm_check_.position_stable)
  {
    res.message = "Position drift is detected.";
    return true;
  }
  if (!pre_arm_check_.position_accurate)
  {
    res.message = "The accuracy of position estimation is too low.";
    return true;
  }
  if (!pre_arm_check_.position_accurate)
  {
    res.message = "The accuracy of orientation estimation is too low.";
    return true;
  }
  if (!pre_arm_check_.position_accurate)
  {
    res.message = "The accuracy of velocity estimation is too low.";
    return true;
  }

  res.message = "Unknown error.";
  return true;
}

void PreArmCheckServer::preArmCheckTimerCb(const rclcpp::TimerEvent& event)
{
  if (battery_ == nullptr || odom_ == nullptr)
    return;

  pre_arm_check_.header.stamp = event.current_real;
  pre_arm_check_.ok = true;

  // バッテリー電圧
  pre_arm_check_.battery_voltage_sufficient = battery_->voltage > drone_.battery.sag_voltage;
  if (!pre_arm_check_.battery_voltage_sufficient)
    pre_arm_check_.ok = false;

  // 姿勢角
  odom_->frame.M.getRPY(roll_, pitch_, yaw_);
  pre_arm_check_.attitude_horizontal = max(abs(roll_), abs(pitch_)) < kAttitudeThresh;
  if (!pre_arm_check_.attitude_horizontal)
    pre_arm_check_.ok = false;

  // 位置のドリフト
  pre_arm_check_.position_stable = true;
  for (size_t i = 0; i < 3; ++i)
  {
    if (!pos_buf_[i].isFilled() || pos_buf_[i].range() > kPosDriftThresh)
    {
      pre_arm_check_.position_stable = false;
      pre_arm_check_.ok = false;
      break;
    }
  }

  // 位置推定の共分散
  const Vector3d pos_cov_diag = odom_->position_covariance.diagonal();
  const auto hor_pos_var = max(pos_cov_diag.x(), pos_cov_diag.y());
  const auto ver_pos_var = pos_cov_diag.z();
  pre_arm_check_.position_accurate =
    hor_pos_var < math::sqr(kHorPosStddevThresh) && ver_pos_var < math::sqr(kVerPosStddevThresh);
  if (!pre_arm_check_.position_accurate)
    pre_arm_check_.ok = false;

  // 姿勢推定の共分散
  const auto rot_var = odom_->orientation_covariance.diagonal().maxCoeff();
  pre_arm_check_.orientation_accurate = rot_var < math::sqr(kRotStddevThresh);
  if (!pre_arm_check_.orientation_accurate)
    pre_arm_check_.ok = false;

  // 速度推定の共分散
  const auto vel_var = odom_->velocity_covariance.diagonal().maxCoeff();
  pre_arm_check_.velocity_accurate = vel_var < math::sqr(kVelStddevThresh);
  if (!pre_arm_check_.velocity_accurate)
    pre_arm_check_.ok = false;

  pre_arm_check_pub_.publish(pre_arm_check_);
}
}  // namespace tobas_pre_arm_check
