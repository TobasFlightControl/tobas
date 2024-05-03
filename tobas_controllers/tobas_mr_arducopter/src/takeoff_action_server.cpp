#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_mr_arducopter/takeoff_action_server.hpp"
#include "../include/tobas_mr_arducopter/constants.hpp"

using namespace std;

namespace tobas_mr_arducopter
{
TakeoffActionServer::TakeoffActionServer(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    as_(nh_, tobas::kTakeoffAction, boost::bind(&self::executeCb, this, _1), false)
{
  set_mode_sc_ = nh_.serviceClient<mavros_msgs::SetMode>(kSetModeSrvName);
  arming_sc_ = nh_.serviceClient<mavros_msgs::CommandBool>(kArmingSrvName);
  takeoff_sc_ = nh_.serviceClient<mavros_msgs::CommandTOL>(kTakeoffSrvName);

  local_pos_sub_ = nh_.subscribe(kLocalPositionPoseTopic, 1, &self::localPositionCb, this);
  param_server_state_sub_ =
    nh_.subscribe(kParamServerStateTopic, 1, &self::paramServerStateCb, this);

  as_.start();
}

bool TakeoffActionServer::isGoalValid(const GoalType& goal)
{
  if (goal.target_altitude < kTakeoffCheckAltThreshold)
  {
    as_.setAborted(
      result_,
      "Target elevation must be greater than " + to_string(kTakeoffCheckAltThreshold) + " [m].");
    return false;
  }

  if (goal.timeout < 0)
  {
    as_.setAborted(result_, "Timeout must be non-negative.");
    return false;
  }

  return true;
}

bool TakeoffActionServer::waitForServiceExistence()
{
  if (!set_mode_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    as_.setAborted(result_, "Failed to connect to '" + kSetModeSrvName + "' service server.");
    return false;
  }

  if (!arming_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    as_.setAborted(result_, "Failed to connect to '" + kArmingSrvName + "' service server.");
    return false;
  }

  if (!takeoff_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    as_.setAborted(result_, "Failed to connect to '" + kTakeoffSrvName + "' service server.");
    return false;
  }

  return true;
}

bool TakeoffActionServer::waitForParamServer(const double& timeout)
{
  while (!is_param_server_ok_)
  {
    if (timeout > 0 && (ros::Time::now() - action_called_time_).toSec() > timeout)
    {
      as_.setAborted(result_, "Timeout while setting flight mode.");
      return false;
    }

    if (as_.isPreemptRequested())
    {
      as_.setPreempted(result_);
      return false;
    }

    TOBAS_INFO_THROTTLE(kRetryInterval, "Waiting for ArduCopter to be ready.");
    ros::Duration(1e-2).sleep();
    ros::spinOnce();
  }

  return true;
}

bool TakeoffActionServer::setMode(const double& timeout)
{
  TOBAS_INFO("Setting flight mode.");

  mavros_msgs::SetMode set_mode_msg;
  set_mode_msg.request.custom_mode = "GUIDED";
  set_mode_msg.response.mode_sent = false;

  while (!set_mode_msg.response.mode_sent)
  {
    if (timeout > 0 && (ros::Time::now() - action_called_time_).toSec() > timeout)
    {
      as_.setAborted(result_, "Timeout while setting flight mode.");
      return false;
    }

    if (as_.isPreemptRequested())
    {
      as_.setPreempted(result_);
      return false;
    }

    if (!set_mode_sc_.call(set_mode_msg))
    {
      TOBAS_WARN("Failed to call '" + kSetModeSrvName + "'. Retrying...");
      ros::Duration(kRetryInterval).sleep();
      ros::spinOnce();
      continue;
    }

    if (!set_mode_msg.response.mode_sent)
    {
      TOBAS_WARN("Failed to send mode. Retrying...");
      ros::Duration(kRetryInterval).sleep();
      ros::spinOnce();
      continue;
    }
  }

  return true;
}

bool TakeoffActionServer::arming(const double& timeout)
{
  TOBAS_INFO("Arming");

  mavros_msgs::CommandBool arming_msg;
  arming_msg.request.value = true;
  arming_msg.response.success = false;

  while (!arming_msg.response.success)
  {
    if (timeout > 0 && (ros::Time::now() - action_called_time_).toSec() > timeout)
    {
      as_.setAborted(result_, "Timeout while arming.");
      return false;
    }

    if (as_.isPreemptRequested())
    {
      as_.setPreempted(result_);
      return false;
    }

    if (!arming_sc_.call(arming_msg))
    {
      TOBAS_WARN("Failed to call '" + kArmingSrvName + "'. Retrying...");
      ros::Duration(kRetryInterval).sleep();
      ros::spinOnce();
      continue;
    }

    if (!arming_msg.response.success)
    {
      TOBAS_WARN("Arming failed. Retrying...");
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
  TOBAS_INFO("Takeoff");

  // リクエストを作成．ヨーは反映されないため高度のみ指定．
  mavros_msgs::CommandTOL takeoff_msg;
  takeoff_msg.request.altitude = target_altitude;

  // 最低1回は実行するためにDo-While文を用いる
  do
  {
    if (timeout > 0 && (ros::Time::now() - action_called_time_).toSec() > timeout)
    {
      as_.setAborted(result_, "Timeout while takeoff.");
      return false;
    }

    if (as_.isPreemptRequested())
    {
      as_.setPreempted(result_);
      return false;
    }

    if (!takeoff_sc_.call(takeoff_msg))
    {
      TOBAS_WARN("Failed to call '" + kTakeoffSrvName + "'. Retrying...");
      ros::Duration(kRetryInterval).sleep();
      ros::spinOnce();
      continue;
    }

    if (!takeoff_msg.response.success)
    {
      TOBAS_WARN("Failed to takeoff. Retrying...");
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
  as_.setSucceeded(result_);
}

void TakeoffActionServer::localPositionCb(const geometry_msgs::PoseStampedConstPtr& pose)
{
  pose_ = pose;
}

void TakeoffActionServer::paramServerStateCb(const std_msgs::BoolConstPtr& state)
{
  is_param_server_ok_ = state->data;
}

void TakeoffActionServer::executeCb(const GoalType::ConstPtr& goal)
{
  TOBAS_INFO("Action is called.");
  action_called_time_ = ros::Time::now();

  if (!isGoalValid(*goal))
    return;

  if (!waitForServiceExistence())
    return;

  if (!waitForParamServer(goal->timeout))
    return;

  if (!setMode(goal->timeout))
    return;

  if (!arming(goal->timeout))
    return;

  if (!takeoff(goal->timeout, goal->target_altitude))
    return;

  setSucceeded();
}
}  // namespace tobas_mr_arducopter
