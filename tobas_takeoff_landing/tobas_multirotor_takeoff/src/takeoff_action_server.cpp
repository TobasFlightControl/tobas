#include <tobas_std_tools/trajectory.hpp>
#include <tobas_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.h>

#include "../include/tobas_multirotor_takeoff/takeoff_action_server.hpp"

using namespace std;

namespace tobas_multirotor_takeoff
{
TakeoffActionServer::TakeoffActionServer(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    as_(nh_, tobas::kTakeoffAction, boost::bind(&self::executeCb, this, _1), false)
{
  getRosParams();

  registerPublishers();
  registerSubscribers();

  as_.start();
}

void TakeoffActionServer::getRosParams()
{
}

void TakeoffActionServer::registerPublishers()
{
  cmd_pub_ = nh_.advertise<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic, 1);
}

void TakeoffActionServer::registerSubscribers()
{
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
}

bool TakeoffActionServer::isGoalValid(const GoalType& goal)
{
  if (goal->target_altitude <= 0)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "Target altitude must be positive.");
    return false;
  }

  if (goal->target_duration <= 0)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "Target duration must be positive.");
    return false;
  }

  return true;
}

void TakeoffActionServer::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  odom_ = odom;
}

void TakeoffActionServer::executeCb(const GoalType& goal)
{
  rosInfo(name_, "Action is called.");

  ros::Rate rate(kUpdateRate);

  while (odom_ == nullptr)
  {
    rosInfoThrottle(kInfoPeriod, name_, "Waiting for " << ns() << tobas::kOdometryTopic);
    ros::spinOnce();
    rate.sleep();
  }

  // Check goal validity
  if (!isGoalValid(goal))
    return;

  // 軌道を生成
  tobas_std::CubicSpline traj_z(odom_->pose.pos.z(), goal->target_altitude, goal->target_duration);

  // 初期状態
  const auto start_time = ros::Time::now();
  const auto start_x = odom_->pose.pos.x();
  const auto start_y = odom_->pose.pos.y();
  const auto start_yaw = odom_->pose.euler.yaw;

  // 軌道を発行
  while (nh_.ok())
  {
    const auto t = (ros::Time::now() - start_time).toSec();

    if (t > traj_z.duration())
    {
      result_.error_code = ResultType::NO_ERROR;
      as_.setSucceeded(result_);
      return;
    }

    if (as_.isPreemptRequested())
    {
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return;
    }

    // コマンドを作成
    const auto cmd = boost::make_shared<tobas_msgs::PosVelAccYaw>();
    cmd->level = goal->level;
    cmd->vel_frame.data = tobas_msgs::FrameId::GLOBAL;
    cmd->acc_frame.data = tobas_msgs::FrameId::GLOBAL;
    cmd->pos.setZero();
    cmd->vel.setZero();
    cmd->acc.setZero();

    // 水平位置とヨー角は初期状態を維持
    cmd->pos.x() = start_x;
    cmd->pos.y() = start_y;
    cmd->yaw = start_yaw;

    // 鉛直方向の軌道を生成
    traj_z.get(t, cmd->pos.z(), cmd->vel.z(), cmd->acc.z());

    // コマンドを発行
    cmd_pub_.publish(cmd);

    ros::spinOnce();
    rate.sleep();
  }
}
}  // namespace tobas_multirotor_takeoff
