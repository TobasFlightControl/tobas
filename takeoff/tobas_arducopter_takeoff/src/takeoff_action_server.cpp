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

  set_mode_ac_ = nh_.serviceClient<mavros_msgs::SetMode>(kSetModeSrvName);
  arming_ac_ = nh_.serviceClient<mavros_msgs::CommandBool>(kArmingSrvName);
  takeoff_ac_ = nh_.serviceClient<mavros_msgs::CommandTOL>(kTakeoffSrvName);

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
  pt_sub_ = nh_.subscribe(tobas::kPoseTwistTopic, 1, &self::poseTwistCb, this, tcpNoDelay());
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

void TakeoffActionServer::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  pt_ = pt;
}

void TakeoffActionServer::executeCb(const GoalType& goal)
{
  rosInfo(name_, "Action is called.");

  const auto start_time = ros::Time::now();
  ResultType result;

  if (pt_ == nullptr)
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(
      result, nh_.getNamespace() + "/" + tobas::kPoseTwistTopic + " is not received yet.");
    return;
  }

  // Check services existence
  if (!set_mode_ac_.waitForExistence(ros::Duration(kWaitForService)))
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(result, "Failed to connect to '" + kSetModeSrvName + "' service server.");
    return;
  }
  if (!arming_ac_.waitForExistence(ros::Duration(kWaitForService)))
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(result, "Failed to connect to '" + kArmingSrvName + "' service server.");
    return;
  }
  if (!takeoff_ac_.waitForExistence(ros::Duration(kWaitForService)))
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(result, "Failed to connect to '" + kTakeoffSrvName + "' service server.");
    return;
  }

  // Set mode
  rosInfo(name_, "Setting flight mode.");
  mavros_msgs::SetMode set_mode_msg;
  set_mode_msg.request.custom_mode = "GUIDED";
  set_mode_msg.response.mode_sent = false;
  while (!set_mode_msg.response.mode_sent)
  {
    if ((ros::Time::now() - start_time).toSec() > kTimeout)
    {
      result.error_code = ResultType::TIMEOUT;
      as_.setAborted(result, "Timeout while setting flight mode.");
      return;
    }

    if (!set_mode_ac_.call(set_mode_msg))
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

  // Arming
  rosInfo(name_, "Arming");
  mavros_msgs::CommandBool arming_msg;
  arming_msg.request.value = true;
  arming_msg.response.success = false;
  while (!arming_msg.response.success)
  {
    if ((ros::Time::now() - start_time).toSec() > kTimeout)
    {
      result.error_code = ResultType::TIMEOUT;
      as_.setAborted(result, "Timeout while arming.");
      return;
    }

    if (!arming_ac_.call(arming_msg))
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

  // Takeoff
  rosInfo(name_, "Takeoff");
  mavros_msgs::CommandTOL takeoff_msg;
  takeoff_msg.request.altitude = kTargetElevation;
  // takeoff_msg.request.yaw = pt_->pose.euler.yaw + M_PI_2;  // ヨーは反映されない
  // 最低1回は実行するためにDo-While文を用いる
  do
  {
    if ((ros::Time::now() - start_time).toSec() > kTimeout)
    {
      result.error_code = ResultType::TIMEOUT;
      as_.setAborted(result, "Timeout while takeoff.");
      return;
    }

    if (!takeoff_ac_.call(takeoff_msg))
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
  } while (pt_->pose.pos.z() < kTakeoffCheckThreshold);

  // Succeeded
  result.error_code = ResultType::NO_ERROR;
  as_.setSucceeded(result);
}
}  // namespace tobas_arducopter_takeoff
