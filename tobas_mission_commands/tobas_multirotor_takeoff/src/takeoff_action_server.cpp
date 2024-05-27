#include <tobas_std_tools/trajectory.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/SetArm.h>

#include "../include/tobas_multirotor_takeoff/takeoff_action_server.hpp"

using namespace std;

namespace tobas_multirotor_takeoff
{
TakeoffActionServer::TakeoffActionServer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), as_(nh_, tobas::kTakeoffAction, boost::bind(&self::executeCb, this, _1), false)
{
  cmd_pub_ = nh_.advertise<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic, 1);
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this);
  set_arm_sc_ = nh_.serviceClient<tobas_msgs::SetArm>(tobas::kSetArmSrv);
  as_.start();
}

bool TakeoffActionServer::isGoalValid(const GoalType& goal)
{
  if (goal.target_altitude <= 0)
  {
    as_.setAborted(result_, "Target altitude must be positive.");
    return false;
  }

  if (goal.altitude_tolerance <= 0)
  {
    as_.setAborted(result_, "Altitude tolerance must be positive.");
    return false;
  }

  if (goal.duration <= 0)
  {
    as_.setAborted(result_, "Target duration must be positive.");
    return false;
  }

  return true;
}

bool TakeoffActionServer::armRotors()
{
  if (!set_arm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    as_.setAborted(result_, "Failed to connect to arming service server.");
    return false;
  }

  tobas_msgs::SetArm set_arm_msg;
  set_arm_msg.request.arming = true;
  if (!set_arm_sc_.call(set_arm_msg) || !set_arm_msg.response.success)
  {
    as_.setAborted(result_, "Failed to arm rotors: " + set_arm_msg.response.message);
    return false;
  }

  return true;
}

void TakeoffActionServer::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  odom_ = odom;
}

void TakeoffActionServer::executeCb(const GoalType::ConstPtr& goal)
{
  TOBAS_INFO("Action is called.");

  // Check goal validity
  if (!isGoalValid(*goal))
    return;

  // Check odometry
  if (odom_ == nullptr)
  {
    as_.setAborted(result_, "Odometry message is not received yet.");
    return;
  }
  if (odom_->status != tobas_msgs::Odometry::NO_ERROR)
  {
    as_.setAborted(result_, "There is a problem with the state estimation.");
    return;
  }

  // Arm rotors
  if (!armRotors())
    return;

  // 軌道を生成
  tobas_std::CubicSpline traj_z(odom_->frame.p.z(), goal->target_altitude, goal->duration);
  const auto duration = traj_z.duration();

  // 初期状態
  const auto start_time = ros::Time::now();
  const auto start_x = odom_->frame.p.x();
  const auto start_y = odom_->frame.p.y();
  const auto start_yaw = tobas_kdl::Euler(odom_->frame.M).yaw;

  // 軌道を発行
  ros::Rate rate(kUpdateRate);
  while (nh_.ok())
  {
    // 開始からの経過時間を計算
    const auto t = (ros::Time::now() - start_time).toSec();

    // タイムアウトの確認
    if (goal->timeout > 0 && t > duration + goal->timeout)
    {
      as_.setAborted(result_, "Timeout before reaching the target altitude.");
      return;
    }

    // コマンドを発行し終え，且つ許容範囲内に入っていたらアクション成功
    const auto alt_error = abs(goal->target_altitude - odom_->frame.p.z());
    if (t > duration && alt_error < goal->altitude_tolerance)
    {
      as_.setSucceeded(result_);
      return;
    }

    // コマンドを作成
    const auto cmd = boost::make_shared<tobas_msgs::PosVelAccYaw>();
    cmd->level = goal->level;
    cmd->frame_id.data = tobas_msgs::FrameId::WORLD;
    cmd->pos.setZero();
    cmd->vel.setZero();
    cmd->acc.setZero();

    // 水平位置とヨー角は初期状態を維持
    cmd->pos.x(start_x);
    cmd->pos.y(start_y);
    cmd->yaw = start_yaw;

    // 鉛直方向の軌道を生成
    traj_z.get(t, cmd->pos.z(), cmd->vel.z(), cmd->acc.z());

    // コマンドを発行
    cmd_pub_.publish(cmd);

    // アクション中止の場合は目標速度・加速度を0にして終了
    if (as_.isPreemptRequested())
    {
      cmd->vel.setZero();
      cmd->acc.setZero();
      cmd_pub_.publish(cmd);

      as_.setPreempted(result_);
      return;
    }

    ros::spinOnce();
    rate.sleep();
  }
}
}  // namespace tobas_multirotor_takeoff
