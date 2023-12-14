#include <std_msgs/Float64.h>
#include <controller_manager_msgs/ListControllers.h>

#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_gazebo/joint_command_handler.hpp"

using namespace std;
using namespace KDL;

namespace tobas_gazebo
{
JointCommandHandler::JointCommandHandler(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const std::string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  registerPublishers();
  registerSubscribers();
}

void JointCommandHandler::getRosParams()
{
}

void JointCommandHandler::registerPublishers()
{
}

void JointCommandHandler::registerSubscribers()
{
  positions_sub_ = nh_.subscribe(
    tobas::kJointPositionsCmdTopic, 1, &self::jointPositionsCmdCb, this, tcpNoDelay());
  velocities_sub_ = nh_.subscribe(
    tobas::kJointVelocitiesCmdTopic, 1, &self::jointVelocitiesCmdCb, this, tcpNoDelay());
  efforts_sub_ =
    nh_.subscribe(tobas::kJointEffortsCmdTopic, 1, &self::jointEffortsCmdCb, this, tcpNoDelay());
}

int JointCommandHandler::initialize()
{
  // ノードの起動順が不確定なため，サービスコールをコンストラクタでやるべきではない
  // 制限時間を設けて成功するまで何度も繰り返すのが正しい
  ros::ServiceClient sc =
    nh_.serviceClient<controller_manager_msgs::ListControllers>(tobas::kListControllersSrv);
  if (!sc.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    rosError(name_, "Failed to connect to '" << tobas::kListControllersSrv << "' service server.");
    return -1;
  }

  controller_manager_msgs::ListControllers msg;
  if (!sc.call(msg))
  {
    rosError(name_, "Failed to call '" << tobas::kListControllersSrv << "'.");
    return -1;
  }

  command_type_t cmd_type;
  for (const auto& item : msg.response.controller)
  {
    if (item.claimed_resources.size() != 1)
      continue;
    if (item.claimed_resources[0].resources.size() != 1)
      continue;

    if (item.type.ends_with("JointPositionController"))
      cmd_type = POSITION;
    else if (item.type.ends_with("JointVelocityController"))
      cmd_type = VELOCITY;
    else if (item.type.ends_with("JointEffortController"))
      cmd_type = EFFORT;
    else
    {
      rosError(name_, "Unknown controller type: " << item.type);
      return -1;
    }

    const auto& jnt_name = item.claimed_resources[0].resources[0];
    const auto topic = item.name + "/command";
    ctrl_map_[jnt_name] = make_pair(cmd_type, nh_.advertise<std_msgs::Float64>(topic, 1));
  }

  return 0;
}

void JointCommandHandler::eventCb(const tobas_msgs::EventConstPtr& event)
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

void JointCommandHandler::jointPositionsCmdCb(const tobas_msgs::JointPositionsConstPtr& positions)
{
  if (positions->name.size() != positions->data.size())
  {
    rosError(name_, "The sizes of name and data in joint positions message do not match.");
    return;
  }

  if (ctrl_map_.size() == 0 && initialize() < 0)
  {
    ctrl_map_.clear();
    return;
  }

  for (size_t i = 0; i < positions->name.size(); ++i)
  {
    const auto& jnt_name = positions->name[i];

    if (!ctrl_map_.contains(jnt_name))
    {
      rosError(name_, "Transmission for joint '" << jnt_name << "' is not found.");
      continue;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == POSITION)
    {
      const auto cmd = boost::make_shared<std_msgs::Float64>();
      cmd->data = positions->data[i];
      pub.publish(cmd);
    }
    else
    {
      rosWarn(
        name_, "Transmission type for joint '"
                 << jnt_name << "' is not position. So received position command for joint '"
                 << jnt_name << "' is ignored.");
    }
  }
}

void JointCommandHandler::jointVelocitiesCmdCb(
  const tobas_msgs::JointVelocitiesConstPtr& velocities)
{
  if (velocities->name.size() != velocities->data.size())
  {
    rosError(name_, "The sizes of name and data in joint velocities message do not match.");
    return;
  }

  if (ctrl_map_.size() == 0 && initialize() < 0)
  {
    ctrl_map_.clear();
    return;
  }

  for (size_t i = 0; i < velocities->name.size(); ++i)
  {
    const auto& jnt_name = velocities->name[i];

    if (!ctrl_map_.contains(jnt_name))
    {
      rosError(name_, "Transmission for joint '" << jnt_name << "' is not found.");
      continue;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == VELOCITY)
    {
      const auto cmd = boost::make_shared<std_msgs::Float64>();
      cmd->data = velocities->data[i];
      pub.publish(cmd);
    }
    else
    {
      rosWarn(
        name_, "Transmission type for joint '"
                 << jnt_name << "' is not velocity. So received velocity command for joint '"
                 << jnt_name << "' is ignored.");
    }
  }
}

void JointCommandHandler::jointEffortsCmdCb(const tobas_msgs::JointEffortsConstPtr& efforts)
{
  if (efforts->name.size() != efforts->data.size())
  {
    rosError(name_, "The sizes of name and data in joint efforts message do not match.");
    return;
  }

  if (ctrl_map_.size() == 0 && initialize() < 0)
  {
    ctrl_map_.clear();
    return;
  }

  for (size_t i = 0; i < efforts->name.size(); ++i)
  {
    const auto& jnt_name = efforts->name[i];

    if (!ctrl_map_.contains(jnt_name))
    {
      rosError(name_, "Transmission for joint '" << jnt_name << "' is not found.");
      continue;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == EFFORT)
    {
      const auto cmd = boost::make_shared<std_msgs::Float64>();
      cmd->data = efforts->data[i];
      pub.publish(cmd);
    }
    else
    {
      rosWarn(
        name_, "Transmission type for joint '"
                 << jnt_name << "' is not effort. So received effort command for joint '"
                 << jnt_name << "' is ignored.");
    }
  }
}
}  // namespace tobas_gazebo
