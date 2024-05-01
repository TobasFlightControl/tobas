#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/trajectory.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_ros_tools/service.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/GetGnssOrigin.h>

#include "../include/tobas_multirotor_move/move_action_server.hpp"

using namespace std;

namespace tobas_multirotor_move
{
MoveActionServer::MoveActionServer(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    as_(nh_, tobas::kTakeoffAction, boost::bind(&self::executeCb, this, _1), false)
{
  cmd_pub_ = nh_.advertise<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic, 1);

  arming_sub_ = nh_.subscribe(tobas::kArmingTopic, 1, &self::armingCb, this);
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this);

  as_.start();
}

bool MoveActionServer::isGoalValid(const GoalType& goal)
{
  if (goal.target_latitude < -90 || 90 < goal.target_latitude)
  {
    result_.success = false;
    as_.setAborted(result_, "Invalid target latitude.");
    return false;
  }

  if (goal.target_longitude < -180 || 180 < goal.target_longitude)
  {
    result_.success = false;
    as_.setAborted(result_, "Invalid target longitude.");
    return false;
  }

  if (goal.acceptance_radius <= 0)
  {
    result_.success = false;
    as_.setAborted(result_, "Acceptance radius must be positive.");
    return false;
  }

  if (goal.duration <= 0)
  {
    result_.success = false;
    as_.setAborted(result_, "Target duration must be positive.");
    return false;
  }

  return true;
}

bool MoveActionServer::getCartPosFromGnss(
  const double& latitude,
  const double& longitude,
  double& x,
  double& y)
{
  // FIXME: XYの絶対値が大きいほど平面近似の誤差が大きくなる
  // 長距離移動の際は目標地点の経緯度を基準にするなどの工夫が必要

  tobas_ros::ServiceClientWrapper<tobas_msgs::GetGnssOrigin> sc(nh_, tobas::kGetGnssOriginSrv);
  if (!sc.call() || !sc.res.success)
  {
    result_.success = false;
    as_.setAborted(result_, "Failed to get GNSS origin.");
    return false;
  }

  const auto& latitude_0 = sc.res.latitude;
  const auto& longitude_0 = sc.res.longitude;
  tobas_std::gpsToCartRelative(latitude, longitude, latitude_0, longitude_0, x, y);

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
    result_.success = false;
    as_.setAborted(result_, "Rotors are disarmed.");
    return;
  }

  // Check if state estimation is OK
  if (odom_ == nullptr || odom_->status != tobas_msgs::Odometry::NO_ERROR)
  {
    result_.success = false;
    as_.setAborted(result_, "There is a problem with the state estimation.");
    return;
  }

  // 現在位置
  const auto& cur_pos = odom_->frame.p;

  // 目標位置
  KDL::Vector tar_pos;
  if (!getCartPosFromGnss(goal->target_latitude, goal->target_longitude, tar_pos.x(), tar_pos.y()))
    return;
  tar_pos.z(goal->target_altitude);  // TODO: 目標高度がMSLで与えられた場合にも対応

  // 軌道を生成
  // TODO: 最高速度を考慮して起動を作成
  tobas_std::CubicSpline traj_x(cur_pos.x(), tar_pos.x(), goal->duration);
  tobas_std::CubicSpline traj_y(cur_pos.y(), tar_pos.y(), goal->duration);
  tobas_std::CubicSpline traj_z(cur_pos.z(), tar_pos.z(), goal->duration);
  const auto duration = tobas_std::max(traj_x.duration(), traj_y.duration(), traj_z.duration());

  // 初期状態
  const auto start_time = ros::Time::now();
  const auto start_yaw = KDL::Euler(odom_->frame.M).yaw;

  // 軌道を発行
  ros::Rate rate(kUpdateRate);
  while (nh_.ok())
  {
    // クライアントからアクション中止のリクエストが来ているか確認
    if (as_.isPreemptRequested())
    {
      result_.success = false;
      as_.setPreempted(result_);
      return;
    }

    // 開始からの経過時間を計算
    const auto t = (ros::Time::now() - start_time).toSec();

    // タイムアウトの確認
    if (goal->timeout > 0 && t > duration + goal->timeout)
    {
      result_.success = false;
      as_.setAborted(result_, "Timeout before reaching the goal position.");
      return;
    }

    // コマンドを発行し終え，且つ許容範囲内に入っていたらミッション成功
    const auto pos_error = tar_pos - cur_pos;
    if (t > duration && pos_error.norm() < goal->acceptance_radius)
    {
      result_.success = true;
      as_.setSucceeded(result_);
      return;
    }

    // コマンドを作成
    const auto cmd = boost::make_shared<tobas_msgs::PosVelAccYaw>();
    cmd->level = goal->level;
    cmd->frame_id.data = tobas_msgs::FrameId::WORLD;

    // ヨー角は初期状態を維持
    cmd->yaw = start_yaw;

    // 現在の時刻における目標状態を取得
    traj_z.get(t, cmd->pos.x(), cmd->vel.x(), cmd->acc.x());
    traj_z.get(t, cmd->pos.y(), cmd->vel.y(), cmd->acc.y());
    traj_z.get(t, cmd->pos.z(), cmd->vel.z(), cmd->acc.z());

    // コマンドを発行
    cmd_pub_.publish(cmd);

    // フィードバックを発行
    const auto feedback = boost::make_shared<FeedbackType>();
    feedback->current_position = cur_pos;
    feedback->target_position = tar_pos;
    feedback->position_error = pos_error;
    as_.publishFeedback(feedback);

    ros::spinOnce();
    rate.sleep();
  }
}
}  // namespace tobas_multirotor_move
