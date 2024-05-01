#include <tobas_kdl/euler.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/SetArm.h>

#include "../include/tobas_multirotor_landing/landing_action_server.hpp"

using namespace std;

namespace tobas_multirotor_landing
{
LandActionServer::LandActionServer(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    is_action_running_(false),
    as_(nh_, tobas::kLandingAction, boost::bind(&self::executeCb, this, _1), false)
{
  cmd_pub_ = nh_.advertise<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic, 1);
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
  set_arm_sc_ = nh_.serviceClient<tobas_msgs::SetArm>(tobas::kSetArmSrv);

  as_.start();
}

void LandActionServer::reset()
{
  odom_ = nullptr;
  is_history_filled_ = false;
  alt_history_.clear();
}

bool LandActionServer::disarmRotors()
{
  if (!set_arm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    result_.error_code = ResultType::DISARM_FAILED;
    as_.setAborted(result_, "Failed to connect to arming service server.");
    return false;
  }

  tobas_msgs::SetArm set_arm_msg;
  set_arm_msg.request.arming = false;
  if (!set_arm_sc_.call(set_arm_msg) || !set_arm_msg.response.success)
  {
    result_.error_code = ResultType::DISARM_FAILED;
    as_.setAborted(result_, "Failed to disarm rotors.");
    return false;
  }

  return true;
}

void LandActionServer::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  if (odom->status != tobas_msgs::Odometry::NO_ERROR)
    return;

  if (!is_action_running_)
    return;

  // 現在の時刻と高度を履歴に追加
  const auto& cur_time = odom->header.stamp;
  const auto& altitude = odom->frame.p.z();
  alt_history_.emplace_back(cur_time, altitude);

  // 古い履歴を削除
  // 要素にアクセスできるように最低1つは残しておく
  while (alt_history_.size() > 2 && (cur_time - alt_history_.front().first).toSec() > kTimeWindow)
  {
    alt_history_.pop_front();
    if (!is_history_filled_)
      is_history_filled_ = true;
  }

  // 最新の状態を更新
  odom_ = odom;
}

void LandActionServer::executeCb(const GoalType::ConstPtr& goal)
{
  TOBAS_INFO("Action is called.");

  reset();
  is_action_running_ = true;

  // 現在の状態を取得
  ros::spinOnce();
  ros::Duration(0.1).sleep();  // ベース状態が更新されるよう少し待機
  if (odom_ == nullptr)
  {
    is_action_running_ = false;
    result_.error_code = ResultType::NOT_READY;
    as_.setAborted(result_, "Failed to get pose & twist.");
    return;
  }

  // 初期状態
  const auto start_time = ros::Time::now();
  const auto start_x = odom_->frame.p.x();
  const auto start_y = odom_->frame.p.y();
  const auto start_z = odom_->frame.p.z();
  const auto start_yaw = KDL::Euler(odom_->frame.M).yaw;

  // 高度チェック
  ros::Rate rate(kUpdateRate);
  while (nh_.ok())
  {
    if (as_.isPreemptRequested())
    {
      is_action_running_ = false;
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return;
    }

    if (is_history_filled_)
    {
      // 一定時間幅の高度が一定の範囲内ならモータを停止して終了
      const auto alt_range = abs(alt_history_.front().second - alt_history_.back().second);
      if (alt_range < kStableAltitudeRange)
      {
        TOBAS_INFO("Landing detected. Stopping motors.");
        if (!disarmRotors())
          return;
        result_.error_code = ResultType::NO_ERROR;
        as_.setSucceeded(result_);
        return;
      }
    }

    // コマンドを作成
    const auto t = (ros::Time::now() - start_time).toSec();
    const auto cmd = boost::make_shared<tobas_msgs::PosVelAccYaw>();
    cmd->level = goal->level;
    cmd->frame_id.data = tobas_msgs::FrameId::WORLD;
    cmd->pos.x(start_x);
    cmd->pos.y(start_y);
    cmd->pos.z(start_z - kVerticalSpeed * t);
    cmd->vel.x(0);
    cmd->vel.y(0);
    cmd->vel.z(-kVerticalSpeed);
    cmd->acc.setZero();
    cmd->yaw = start_yaw;

    // コマンドを発行
    cmd_pub_.publish(cmd);

    ros::spinOnce();
    rate.sleep();
  }
}
}  // namespace tobas_multirotor_landing
