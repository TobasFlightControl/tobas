#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>

#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/VelocityYaw.h>

#include "../include/tobas_arducopter_takeoff/takeoff_action_server.hpp"

using namespace std;

namespace tobas_arducopter_takeoff
{
TakeoffActionServer::TakeoffActionServer(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name),
    as_(nh_, tobas::kTakeoffAction, boost::bind(&self::executeCb, this, _1), false)
{
  getRosParams();

  set_mode_sc_ = nh_.serviceClient<mavros_msgs::SetMode>(kSetModeSrvName);
  arming_sc_ = nh_.serviceClient<mavros_msgs::CommandBool>(kArmingSrvName);
  takeoff_sc_ = nh_.serviceClient<mavros_msgs::CommandTOL>(kTakeoffSrvName);

  registerPublishers();
  registerSubscribers();

  as_.start();
}

void TakeoffActionServer::getRosParams()
{
}

void TakeoffActionServer::registerPublishers()
{
}

void TakeoffActionServer::registerSubscribers()
{
  event_sub_ = nh_.subscribe(tobas::kEventTopic, 1, &self::eventCb, this, tcpNoDelay());
  local_pos_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &self::localPositionCb, this);
}

bool TakeoffActionServer::isGoalValid(const GoalType& goal)
{
  if (goal->target_altitude < kTakeoffCheckAltThreshold)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(
      result_,
      "Target elevation must be greater than " + to_string(kTakeoffCheckAltThreshold) + " [m].");
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

bool TakeoffActionServer::waitForServiceExistence()
{
  if (!set_mode_sc_.waitForExistence(ros::Duration(kWaitForService)))
  {
    result_.error_code = ResultType::NOT_READY;
    as_.setAborted(result_, "Failed to connect to '" + kSetModeSrvName + "' service server.");
    return false;
  }

  if (!arming_sc_.waitForExistence(ros::Duration(kWaitForService)))
  {
    result_.error_code = ResultType::NOT_READY;
    as_.setAborted(result_, "Failed to connect to '" + kArmingSrvName + "' service server.");
    return false;
  }

  if (!takeoff_sc_.waitForExistence(ros::Duration(kWaitForService)))
  {
    result_.error_code = ResultType::NOT_READY;
    as_.setAborted(result_, "Failed to connect to '" + kTakeoffSrvName + "' service server.");
    return false;
  }

  return true;
}

bool TakeoffActionServer::waitForPoseReceived(const double& timeout)
{
  while (pose_ == nullptr)
  {
    if ((ros::Time::now() - action_called_time_).toSec() > timeout)
    {
      result_.error_code = ResultType::TIMEOUT;
      as_.setAborted(result_, "Timeout while setting flight mode.");
      return false;
    }

    if (as_.isPreemptRequested())
    {
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return false;
    }

    rosInfoThrottle(kRetryInterval, name_, "Waiting for ArduCopter to be ready.");
    ros::Duration(1e-2).sleep();
    ros::spinOnce();
  }

  return true;
}

bool TakeoffActionServer::setMode(const double& timeout)
{
  rosInfo(name_, "Setting flight mode.");

  mavros_msgs::SetMode set_mode_msg;
  set_mode_msg.request.custom_mode = "GUIDED";
  set_mode_msg.response.mode_sent = false;

  while (!set_mode_msg.response.mode_sent)
  {
    if ((ros::Time::now() - action_called_time_).toSec() > timeout)
    {
      result_.error_code = ResultType::TIMEOUT;
      as_.setAborted(result_, "Timeout while setting flight mode.");
      return false;
    }

    if (as_.isPreemptRequested())
    {
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return false;
    }

    if (!set_mode_sc_.call(set_mode_msg))
    {
      rosWarn(name_, "Failed to call '" + kSetModeSrvName + "'. Retrying...");
      ros::Duration(kRetryInterval).sleep();
      ros::spinOnce();
      continue;
    }

    if (!set_mode_msg.response.mode_sent)
    {
      rosWarn(name_, "Failed to send mode. Retrying...");
      ros::Duration(kRetryInterval).sleep();
      ros::spinOnce();
      continue;
    }
  }

  return true;
}

bool TakeoffActionServer::arming(const double& timeout)
{
  rosInfo(name_, "Arming");

  mavros_msgs::CommandBool arming_msg;
  arming_msg.request.value = true;
  arming_msg.response.success = false;

  while (!arming_msg.response.success)
  {
    if ((ros::Time::now() - action_called_time_).toSec() > timeout)
    {
      result_.error_code = ResultType::TIMEOUT;
      as_.setAborted(result_, "Timeout while arming.");
      return false;
    }

    if (as_.isPreemptRequested())
    {
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return false;
    }

    if (!arming_sc_.call(arming_msg))
    {
      rosWarn(name_, "Failed to call '" + kArmingSrvName + "'. Retrying...");
      ros::Duration(kRetryInterval).sleep();
      ros::spinOnce();
      continue;
    }

    if (!arming_msg.response.success)
    {
      rosWarn(name_, "Arming failed. Retrying...");
      ros::Duration(kRetryInterval).sleep();
      ros::spinOnce();
      continue;
    }
  }

  ros::Duration(kWaitForArming).sleep();  // Armに若干時間がかかるため待機
  return true;
}

bool TakeoffActionServer::takeoff(const double& timeout, const double& target_altitude)
{
  rosInfo(name_, "Takeoff");

  // リクエストを作成．ヨーは反映されないため高度のみ指定．
  mavros_msgs::CommandTOL takeoff_msg;
  takeoff_msg.request.altitude = target_altitude;

  // 最低1回は実行するためにDo-While文を用いる
  do
  {
    if ((ros::Time::now() - action_called_time_).toSec() > timeout)
    {
      result_.error_code = ResultType::TIMEOUT;
      as_.setAborted(result_, "Timeout while takeoff.");
      return false;
    }

    if (as_.isPreemptRequested())
    {
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return false;
    }

    if (!takeoff_sc_.call(takeoff_msg))
    {
      rosWarn(name_, "Failed to call '" + kTakeoffSrvName + "'. Retrying...");
      ros::Duration(kRetryInterval).sleep();
      ros::spinOnce();
      continue;
    }

    if (!takeoff_msg.response.success)
    {
      rosWarn(name_, "Failed to takeoff. Retrying...");
      ros::Duration(kRetryInterval).sleep();
      ros::spinOnce();
      continue;
    }

    ros::Duration(kRetryInterval).sleep();
    ros::spinOnce();
  } while (pose_->pose.position.z < kTakeoffCheckAltThreshold);

  return true;
}

void TakeoffActionServer::setSucceeded()
{
  result_.error_code = ResultType::NO_ERROR;
  as_.setSucceeded(result_);
}

void TakeoffActionServer::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void TakeoffActionServer::localPositionCb(const geometry_msgs::PoseStampedConstPtr& pose)
{
  pose_ = pose;
}

void TakeoffActionServer::executeCb(const GoalType& goal)
{
  rosInfo(name_, "Action is called.");
  action_called_time_ = ros::Time::now();

  if (!isGoalValid(goal))
    return;

  if (!waitForServiceExistence())
    return;

  if (!waitForPoseReceived(goal->timeout))
    return;

  if (!setMode(goal->timeout))
    return;

  if (!arming(goal->timeout))
    return;

  if (!takeoff(goal->timeout, goal->target_altitude))
    return;

  setSucceeded();
}
}  // namespace tobas_arducopter_takeoff
