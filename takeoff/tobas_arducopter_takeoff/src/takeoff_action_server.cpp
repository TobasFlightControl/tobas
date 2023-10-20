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

  set_mode_req_.custom_mode = "GUIDED";
  arming_req_.value = true;
  takeoff_req_.altitude = kTargetElevation;

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

void TakeoffActionServer::executeCb(const GoalType& goal)
{
  rosInfo(name_, "Action is called.");

  ResultType result;

  // Set mode
  if (!set_mode_ac_.waitForExistence(ros::Duration(kWaitForService)))
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(result, "Failed to connect to '" + kSetModeSrvName + "' service server.");
    return;
  }
  if (!set_mode_ac_.call(set_mode_req_, set_mode_res_))
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(result, "Failed to call '" + kSetModeSrvName + "'.");
    return;
  }
  if (!set_mode_res_.mode_sent)
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(result, "Failed to set flight mode.");
    return;
  }

  // Arming
  if (!arming_ac_.waitForExistence(ros::Duration(kWaitForService)))
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(result, "Failed to connect to '" + kArmingSrvName + "' service server.");
    return;
  }
  if (!arming_ac_.call(arming_req_, arming_res_))
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(result, "Failed to call '" + kArmingSrvName + "'.");
    return;
  }
  if (!arming_res_.success)
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(result, "Failed to arm.");
    return;
  }

  // Takeoff
  if (!takeoff_ac_.waitForExistence(ros::Duration(kWaitForService)))
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(result, "Failed to connect to '" + kTakeoffSrvName + "' service server.");
    return;
  }
  if (!takeoff_ac_.call(takeoff_req_, takeoff_res_))
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(result, "Failed to call '" + kTakeoffSrvName + "'.");
    return;
  }
  if (!takeoff_res_.success)
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(result, "Failed to takeoff.");
    return;
  }

  // Succeeded
  result.error_code = ResultType::NO_ERROR;
  as_.setSucceeded(result);
}
}  // namespace tobas_arducopter_takeoff
