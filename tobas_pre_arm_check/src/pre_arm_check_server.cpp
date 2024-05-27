#include <tobas_std_tools/math.hpp>
#include <tobas_ros_tools/time.hpp>
#include <tobas_ros_tools/eigen_conversion.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_pre_arm_check/pre_arm_check_server.hpp"

using namespace std;
using namespace tobas_std;

namespace tobas_pre_arm_check
{
PreArmCheckServer::PreArmCheckServer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name),
    pos_buf_{ TimestampedBufferDouble(kPosDriftCheckTimeWindow), TimestampedBufferDouble(kPosDriftCheckTimeWindow),
              TimestampedBufferDouble(kPosDriftCheckTimeWindow) }
{
  drone_.loadFromParam(nh_);

  pre_arm_check_pub_ = nh_.advertise<tobas_msgs::PreArmCheck>(tobas::kPreArmCheckTopic, 1, true);

  battery_sub_ = nh_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this);
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this);

  pre_arm_check_ss_ = nh_.advertiseService(tobas::kPreArmCheckSrv, &self::preArmCheckSrvCb, this);

  pre_arm_check_timer_ = nh_.createTimer(kPreArmCheckTimerRate, &self::preArmCheckTimerCb, this);
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
  if ((odom->header.stamp - odom_->header.stamp).toSec() < kOdomCallbackInterval)
    return;

  odom_ = odom;

  const auto stamp = tobas_ros::chronoFromRosTime(odom->header.stamp);
  for (size_t i = 0; i < 3; ++i)
    pos_buf_[i].add(stamp, odom->frame.p(i));
}

bool PreArmCheckServer::preArmCheckSrvCb(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res)
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

void PreArmCheckServer::preArmCheckTimerCb(const ros::TimerEvent& event)
{
  if (battery_ == nullptr || odom_ == nullptr)
    return;

  pre_arm_check_.header.stamp = event.current_real;
  pre_arm_check_.ok = true;

  // バッテリー電圧
  pre_arm_check_.battery_voltage_sufficient = battery_->voltage > drone_.batteryConfig().sag_voltage;
  if (!pre_arm_check_.battery_voltage_sufficient)
    pre_arm_check_.ok = false;

  // 姿勢角
  odom_->frame.M.getRPY(roll_, pitch_, yaw_);
  pre_arm_check_.attitude_horizontal = max(abs(roll_), abs(pitch_)) < kAttitudeThreshold;
  if (!pre_arm_check_.attitude_horizontal)
    pre_arm_check_.ok = false;

  // 位置のドリフト
  pre_arm_check_.position_stable = true;
  for (size_t i = 0; i < 3; ++i)
  {
    if (!pos_buf_[i].isFilled() || pos_buf_[i].range() > kPosDriftThreshold)
    {
      pre_arm_check_.position_stable = false;
      pre_arm_check_.ok = false;
      break;
    }
  }

  // 位置推定の共分散
  tobas_ros::matrix3MsgToEigen(odom_->position_covariance, cov_);
  const auto hor_pos_var = max(cov_(0, 0), cov_(1, 1));
  const auto ver_pos_var = cov_(2, 2);
  pre_arm_check_.position_accurate =
    hor_pos_var < sqr(kHorPosStddevThreshold) && ver_pos_var < sqr(kVerPosStddevThreshold);
  if (!pre_arm_check_.position_accurate)
    pre_arm_check_.ok = false;

  // 姿勢推定の共分散
  tobas_ros::matrix3MsgToEigen(odom_->orientation_covariance, cov_);
  const auto rot_var = cov_.diagonal().maxCoeff();
  pre_arm_check_.orientation_accurate = rot_var < sqr(kRotStddevThreshold);
  if (!pre_arm_check_.orientation_accurate)
    pre_arm_check_.ok = false;

  // 速度推定の共分散
  tobas_ros::matrix3MsgToEigen(odom_->linear_velocity_covariance, cov_);
  const auto vel_var = cov_.diagonal().maxCoeff();
  pre_arm_check_.velocity_accurate = vel_var < sqr(kVelStddevThreshold);
  if (!pre_arm_check_.velocity_accurate)
    pre_arm_check_.ok = false;

  pre_arm_check_pub_.publish(pre_arm_check_);
}
}  // namespace tobas_pre_arm_check
