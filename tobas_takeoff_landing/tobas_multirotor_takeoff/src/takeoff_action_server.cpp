#include <tobas_std_tools/trajectory.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/util.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/SetArm.h>

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

  set_arm_sc_ = nh_.serviceClient<tobas_msgs::SetArm>(tobas::kSetArmSrv);

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
}

bool TakeoffActionServer::isGoalValid(const GoalType& goal)
{
  if (goal.target_altitude <= 0)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "Target altitude must be positive.");
    return false;
  }

  if (goal.target_duration <= 0)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "Target duration must be positive.");
    return false;
  }

  return true;
}

bool TakeoffActionServer::getStartOdom()
{
  if (!tobas_ros::subscribeOnce(start_odom_, tobas::kOdometryTopic, nh_))
  {
    result_.error_code = ResultType::NOT_READY;
    as_.setAborted(result_, "Failed to get the odometry of the start position.");
    return false;
  }

  return true;
}

bool TakeoffActionServer::armRotors()
{
  if (!set_arm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    result_.error_code = ResultType::NOT_READY;
    as_.setAborted(result_, "Failed to connect to arming service server.");
    return false;
  }

  tobas_msgs::SetArm set_arm_msg;
  set_arm_msg.request.arming = true;
  if (!set_arm_sc_.call(set_arm_msg) || !set_arm_msg.response.success)
  {
    result_.error_code = ResultType::NOT_READY;
    as_.setAborted(result_, "Failed to arm rotors.");
    return false;
  }

  return true;
}

void TakeoffActionServer::executeCb(const GoalType::ConstPtr& goal)
{
  rosInfo(name_, "Action is called.");

  // Check goal validity
  if (!isGoalValid(*goal))
    return;

  // Get the start position
  if (!getStartOdom())
    return;

  // Arm rotors
  if (!armRotors())
    return;

  // 軌道を生成
  tobas_std::CubicSpline traj_z(
    start_odom_.frame.p.z(), goal->target_altitude, goal->target_duration);

  // 初期状態
  const auto start_time = ros::Time::now();
  const auto start_x = start_odom_.frame.p.x();
  const auto start_y = start_odom_.frame.p.y();
  const auto start_yaw = KDL::Euler(start_odom_.frame.M).yaw;

  // 軌道を発行
  ros::Rate rate(kUpdateRate);
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
