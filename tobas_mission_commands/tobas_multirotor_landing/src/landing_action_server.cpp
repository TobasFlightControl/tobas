#include <tobas_ros2_tools/time.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.hpp>
#include <tobas_msgs/SetArm.h>

#include "../include/tobas_multirotor_landing/landing_action_server.hpp"

using namespace std;

namespace tobas_multirotor_landing
{
LandActionServer::LandActionServer(const rclcpp::NodeOptions& options)
  : super(name, options),
    alt_buf_(kTimeWindow),
    as_(node_, tobas::kLandAction, std::bind(&self::executeCb, this, _1), false)
{
  cmd_pub_ = createPublisher<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  set_arm_sc_ = node_.serviceClient<tobas_msgs::SetArm>(tobas::kSetArmSrv);

  as_.start();
}

bool LandActionServer::disarmRotors()
{
  if (!set_arm_sc_.wait_for_service(rclcpp::Duration(tobas::kWaitForServiceExistence)))
  {
    as_.setAborted(result_, "Failed to connect to arming service server.");
    return false;
  }

  tobas_msgs::SetArm set_arm_msg;
  set_arm_msg.request.arming = false;
  if (!set_arm_sc_.call(set_arm_msg) || !set_arm_msg.response.success)
  {
    as_.setAborted(result_, "Failed to disarm rotors.");
    return false;
  }

  return true;
}

void LandActionServer::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  if (odom->status != tobas_msgs::msg::Odometry::NO_ERROR)
    return;

  odom_ = odom;

  if (!is_action_running_)
    return;

  // 現在の時刻と高度を履歴に追加
  const auto cur_time = ros2::chronoFromRosTime(odom->header.stamp);
  const auto& altitude = odom->frame.p.z();
  alt_buf_.add(cur_time, altitude);
}

void LandActionServer::executeCb(const GoalType::ConstSharedPtr& goal)
{
  TOBAS_INFO("Action is called.");

  // オドメトリが発行されていることを確認
  if (odom_ == nullptr)
  {
    as_.setAborted(result_, "Odometry is not received yet.");
    return;
  }

  // 高度のバッファを初期化
  alt_buf_.clear();

  // 初期状態
  const auto start_time = get_clock()->now();
  const auto start_x = odom_->frame.p.x();
  const auto start_y = odom_->frame.p.y();
  const auto start_z = odom_->frame.p.z();
  const auto start_yaw = kdl::Euler(odom_->frame.M).yaw;

  // Now the action is running
  is_action_running_ = true;

  // 高度チェック
  rclcpp::Rate rate(kUpdateRate);
  while (node_.ok())
  {
    if (alt_buf_.isFilled())
    {
      // 一定時間幅の高度が一定の範囲内ならモータを停止して終了
      // FIXME: 着陸判定が甘い．IMU等も利用してより正確に判定しないと危険．
      const auto alt_range = abs(alt_buf_.firstValue() - alt_buf_.lastValue());
      if (alt_range < kStableAltitudeRange)
      {
        TOBAS_INFO("Landing detected. Stopping motors.");
        is_action_running_ = false;
        if (!disarmRotors())
          return;
        as_.setSucceeded(result_);
        return;
      }
    }

    // コマンドを作成
    const auto t = (get_clock()->now() - start_time).seconds();
    const auto cmd =std::make_unique<tobas_msgs::PosVelAccYaw>();
    cmd->level = goal->level;
    cmd->frame_id.data = tobas_msgs::msg::FrameId::WORLD;
    cmd->pos.x(start_x);
    cmd->pos.y(start_y);
    cmd->pos.z(start_z - kVerticalSpeed * t);
    cmd->vel.x(0);
    cmd->vel.y(0);
    cmd->vel.z(-kVerticalSpeed);
    cmd->acc.setZero();
    cmd->yaw = start_yaw;

    // コマンドを発行
    cmd_pub_->publish(cmd);

    // アクション中止の場合は目標速度・加速度を0にして終了
    if (as_.isPreemptRequested())
    {
      cmd->vel.setZero();
      cmd->acc.setZero();
      cmd_pub_->publish(cmd);

      as_.setPreempted(result_);
      is_action_running_ = false;
      return;
    }

    rclcpp::spinOnce();
    rate.sleep();
  }

  is_action_running_ = false;
}
}  // namespace tobas_multirotor_landing
