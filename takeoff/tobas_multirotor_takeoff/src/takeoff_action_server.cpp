#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/VelocityYaw.h>

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
  cmd_pub_ = nh_.advertise<tobas_msgs::VelocityYaw>(tobas::kVelocityYawCmdTopic, 1);
}

void TakeoffActionServer::registerSubscribers()
{
  super::registerSubscribers();

  pt_sub_ = nh_.subscribe(tobas::kPoseTwistTopic, 1, &self::poseTwistCb, this, tcpNoDelay());
}

bool TakeoffActionServer::isGoalValid(const GoalType& goal)
{
  if (goal->target_altitude <= 0)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "Target altitude must be positive.");
    return false;
  }

  if (goal->target_elevation_speed <= 0)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "Target elevation speed must be positive.");
    return false;
  }

  if (goal->timeout <= 0)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "Timeout must be positive.");
    return false;
  }

  return true;
}

void TakeoffActionServer::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      as_.shutdown();
      break;
    default:
      break;
  }
}

void TakeoffActionServer::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  pt_ = pt;
}

void TakeoffActionServer::executeCb(const GoalType& goal)
{
  rosInfo(name_, "Action is called.");

  if (pt_ == nullptr)
  {
    result_.error_code = ResultType::NOT_READY;
    as_.setAborted(
      result_, nh_.getNamespace() + "/" + tobas::kPoseTwistTopic + " is not received yet.");
    return;
  }

  // Check goal validity
  if (!isGoalValid(goal))
    return;

  // 離陸コマンドを作成
  const auto cmd = boost::make_shared<tobas_msgs::VelocityYaw>();
  cmd->level = goal->level;
  cmd->frame_id.data = tobas_msgs::FrameId::GLOBAL;
  cmd->vel.z(goal->target_elevation_speed);
  cmd->yaw = pt_->pose.euler.yaw;  // yawはアクションが呼ばれたときの値を維持する

  // 離陸コマンドを発行
  cmd_pub_.publish(cmd);

  // 初期状態
  const auto start_alt = pt_->pose.pos.z();
  const auto start_time = ros::Time::now();

  // 高度チェック
  ros::Rate rate(kUpdateRate);
  while (nh_.ok())
  {
    if ((ros::Time::now() - start_time).toSec() > goal->timeout)
    {
      result_.error_code = ResultType::TIMEOUT;
      as_.setAborted(result_, "Timeout while takeoff.");
      return;
    }

    if (as_.isPreemptRequested())
    {
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return;
    }

    // 目標高度に到達したら停止して終了
    if (pt_->pose.pos.z() - start_alt > goal->target_altitude)
    {
      rosInfo(name_, "Target altitude is reached.");

      cmd->vel.z(0.);
      cmd_pub_.publish(cmd);

      result_.error_code = ResultType::NO_ERROR;
      as_.setSucceeded(result_);
      return;
    }

    ros::spinOnce();
    rate.sleep();
  }
}
}  // namespace tobas_multirotor_takeoff
