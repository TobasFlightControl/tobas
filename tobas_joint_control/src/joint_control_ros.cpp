#include <std_msgs/Float64.h>
#include <controller_manager_msgs/ListControllers.h>

#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/util.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_joint_control/joint_control_ros.hpp"

using namespace std;
using namespace KDL;

namespace tobas_joint_control
{
JointControlRos::JointControlRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const std::string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  getCommandTypes();
  registerPublishers();
  registerSubscribers();
}

void JointControlRos::getRosParams()
{
}

void JointControlRos::registerPublishers()
{
  for (const auto& [jnt_name, ctrl_info] : ctrl_info_map_)
  {
    const auto topic = ctrl_info.controller_name + "/command";
    cmd_pub_map_[jnt_name] = nh_.advertise<std_msgs::Float64>(topic, 1);
  }
}

void JointControlRos::registerSubscribers()
{
  js_sub_ =
    nh_.subscribe(tobas::kJointStatesCmdTopic, 1, &self::jointStateCmdCb, this, tcpNoDelay());
}

void JointControlRos::getCommandTypes()
{
  ros::ServiceClient sc =
    nh_.serviceClient<controller_manager_msgs::ListControllers>(tobas::kListControllersSrv);
  if (!sc.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
    ROS_THROW_NAMED(
      name_, "Failed to connect to '" << tobas::kListControllersSrv << "' service server.");

  controller_manager_msgs::ListControllers msg;
  if (!sc.call(msg))
    ROS_THROW_NAMED(name_, "Failed to call '" << tobas::kListControllersSrv << "'.");

  for (const auto& item : msg.response.controller)
  {
    if (item.claimed_resources.size() != 1)
      return;
    if (item.claimed_resources[0].resources.size() != 1)
      return;

    JointControlInfo info;
    info.controller_name = item.name;

    if (item.type.ends_with("JointPositionController"))
      info.command_type = JointControlInfo::POSITION;
    else if (item.type.ends_with("JointVelocityController"))
      info.command_type = JointControlInfo::VELOCITY;
    else if (item.type.ends_with("JointEffortController"))
      info.command_type = JointControlInfo::EFFORT;
    else
    {
      rosError(name_, "Unknown controller type: " << item.type);
      continue;
    }

    const auto& jnt_name = item.claimed_resources[0].resources[0];
    ctrl_info_map_[jnt_name] = info;
  }
}

void JointControlRos::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void JointControlRos::jointStateCmdCb(const sensor_msgs::JointStateConstPtr& js)
{
  if (!dh_ros::isFieldSizeMatch(*js))
  {
    rosError(name_, "The sizes of each field in JointState do not match.");
    return;
  }

  for (size_t i = 0; i < js->name.size(); ++i)
  {
    const auto& jnt_name = js->name[i];

    if (!ctrl_info_map_.contains(jnt_name))
    {
      rosError(name_, "Controller for joint " << jnt_name << " is not found.");
      continue;
    }

    const auto cmd = boost::make_shared<std_msgs::Float64>();
    switch (ctrl_info_map_[jnt_name].command_type)
    {
      case JointControlInfo::POSITION:
        cmd->data = js->position[i];
        break;
      case JointControlInfo::VELOCITY:
        cmd->data = js->velocity[i];
        break;
      case JointControlInfo::EFFORT:
        cmd->data = js->effort[i];
        break;
      default:
        rosError(name_, "Unknown command type: " << ctrl_info_map_[jnt_name].command_type);
        continue;
    }

    cmd_pub_map_[jnt_name].publish(cmd);
  }
}
}  // namespace tobas_joint_control
