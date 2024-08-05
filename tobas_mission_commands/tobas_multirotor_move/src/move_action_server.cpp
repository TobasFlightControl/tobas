#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/trajectory.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_ros2_tools/service.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.hpp>
#include <tobas_msgs/GetGnssOrigin.h>

#include "../include/tobas_multirotor_move/move_action_server.hpp"

using namespace std;

namespace tobas_multirotor_move
{
MoveActionServer::MoveActionServer(, const string& name)
  : super(node, pnh, name), as_(node_, tobas::kMoveAction, std::bind(&self::executeCb, this, _1), false)
{
  cmd_pub_ = node_.advertise<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic, 1);

  arming_sub_ = node_.subscribe(tobas::kArmingTopic, 1, &self::armingCb, this);
  odom_sub_ = node_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this);

  as_.start();
}

bool MoveActionServer::isGoalValid(const GoalType& goal)
{
  if (goal.target_latitude < -90 || 90 < goal.target_latitude)
  {
    as_.setAborted(result_, "Invalid target latitude.");
    return false;
  }

  if (goal.target_longitude < -180 || 180 < goal.target_longitude)
  {
    as_.setAborted(result_, "Invalid target longitude.");
    return false;
  }

  if (goal.acceptance_radius <= 0)
  {
    as_.setAborted(result_, "Acceptance radius must be positive.");
    return false;
  }

  if (goal.duration <= 0)
  {
    as_.setAborted(result_, "Target duration must be positive.");
    return false;
  }

  return true;
}

bool MoveActionServer::computeGoalPosition(const GoalType& goal, kdl::Vector& goal_pos)
{
  // XY軸
  // FIXME: 平面近似誤差が無視できない場合は目標地点の経緯度を基準にするなどの工夫が必要
  ros2::ServiceClientWrapper<tobas_msgs::GetGnssOrigin> sc(node_, tobas::kGetGnssOriginSrv);
  if (!sc.call() || !sc.res.success)
  {
    as_.setAborted(result_, "Failed to get GNSS origin.");
    return false;
  }
  const auto& tar_lat = goal.target_latitude;
  const auto& tar_lon = goal.target_longitude;
  const auto& lat_0 = sc.res.latitude;
  const auto& lon_0 = sc.res.longitude;
  tobas_std::gpsToCartRelative(tar_lat, tar_lon, lat_0, lon_0, goal_pos.x(), goal_pos.y());

  // Z軸
  // TODO: 目標高度がMSLで与えられた場合にも対応
  goal_pos.z(goal.target_altitude);

  return true;
}

void MoveActionServer::armingCb(const std_msgs::BoolConstPtr& arming)
{
  arming_ = arming;
}

void MoveActionServer::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  odom_ = odom;
}

void MoveActionServer::executeCb(const GoalType::ConstPtr& goal)
{
  TOBAS_INFO("Action is called.");

  // Check goal validity
  if (!isGoalValid(*goal))
    return;

  // Check if rotors are armed
  if (arming_ == nullptr || !arming_->data)
  {
    as_.setAborted(result_, "Rotors are disarmed.");
    return;
  }

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

  // 目標位置
  kdl::Vector goal_pos;
  if (!computeGoalPosition(*goal, goal_pos))
    return;

  // 軌道を生成
  // TODO: 最高速度を考慮して起動を作成
  tobas_std::CubicSpline traj_x(odom_->frame.p.x(), goal_pos.x(), goal->duration);
  tobas_std::CubicSpline traj_y(odom_->frame.p.y(), goal_pos.y(), goal->duration);
  tobas_std::CubicSpline traj_z(odom_->frame.p.z(), goal_pos.z(), goal->duration);
  const auto duration = algo::max(traj_x.duration(), traj_y.duration(), traj_z.duration());

  // 初期状態
  const auto start_time = node->get_clock()->now();
  const auto start_yaw = kdl::Euler(odom_->frame.M).yaw;

  // 軌道を発行
  rclcpp::Rate rate(kUpdateRate);
  while (node_.ok())
  {
    // 開始からの経過時間を計算
    const auto t = (node->get_clock()->now() - start_time).seconds();

    // タイムアウトの確認
    if (goal->timeout > 0 && t > duration + goal->timeout)
    {
      as_.setAborted(result_, "Timeout before reaching the goal position.");
      return;
    }

    // コマンドを発行し終え，且つ許容範囲内に入っていたらアクション成功
    const auto& cur_pos = odom_->frame.p;
    const auto pos_error = goal_pos - cur_pos;
    if (t > duration && pos_error.norm() < goal->acceptance_radius)
    {
      as_.setSucceeded(result_);
      return;
    }

    // コマンドを作成
    const auto cmd = make_unique<tobas_msgs::PosVelAccYaw>();
    cmd->level = goal->level;
    cmd->frame_id.data = tobas_msgs::FrameId::WORLD;

    // ヨー角は初期状態を維持
    cmd->yaw = start_yaw;

    // 現在の時刻における目標状態を取得
    traj_x.get(t, cmd->pos.x(), cmd->vel.x(), cmd->acc.x());
    traj_y.get(t, cmd->pos.y(), cmd->vel.y(), cmd->acc.y());
    traj_z.get(t, cmd->pos.z(), cmd->vel.z(), cmd->acc.z());

    // コマンドを発行
    cmd_pub_.publish(cmd);

    // フィードバックを発行
    const auto feedback = make_unique<FeedbackType>();
    feedback->current_position = cur_pos;
    feedback->target_position = cmd->pos;
    feedback->position_error = cmd->pos - cur_pos;
    as_.publishFeedback(feedback);

    // アクション中止の場合は目標速度・加速度を0にして終了
    if (as_.isPreemptRequested())
    {
      cmd->vel.setZero();
      cmd->acc.setZero();
      cmd_pub_.publish(cmd);

      as_.setPreempted(result_);
      return;
    }

    rclcpp::spinOnce();
    rate.sleep();
  }
}
}  // namespace tobas_multirotor_move
