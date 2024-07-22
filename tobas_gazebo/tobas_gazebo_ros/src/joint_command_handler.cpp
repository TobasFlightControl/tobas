#include <std_msgs/Float64.h>
#include <controller_manager_msgs/ListControllers.h>

#include <tobas_std_tools/unordered_map.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_gazebo_ros/joint_command_handler.hpp"

using namespace std;

namespace tobas_gazebo_ros
{
JointCommandHandler::JointCommandHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  positions_sub_ = nh_.subscribe(tobas::kJointPositionsCmdTopic, 1, &self::jointPositionsCmdCb, this, tcpNoDelay());
  velocities_sub_ = nh_.subscribe(tobas::kJointVelocitiesCmdTopic, 1, &self::jointVelocitiesCmdCb, this, tcpNoDelay());
  efforts_sub_ = nh_.subscribe(tobas::kJointEffortsCmdTopic, 1, &self::jointEffortsCmdCb, this, tcpNoDelay());
}

int JointCommandHandler::initialize()
{
  // ノードの起動順が不確定なため，サービスコールをコンストラクタでやるべきではない
  // 制限時間を設けて成功するまで何度も繰り返すのが正しい
  ros::ServiceClient sc = nh_.serviceClient<controller_manager_msgs::ListControllers>(tobas::kListControllersSrv);
  if (!sc.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    TOBAS_ERROR("Failed to connect to '", tobas::kListControllersSrv, "' service server.");
    return -1;
  }

  controller_manager_msgs::ListControllers msg;
  if (!sc.call(msg))
  {
    TOBAS_ERROR("Failed to call '", tobas::kListControllersSrv, "'.");
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
      TOBAS_ERROR("Unknown controller type: ", item.type);
      return -1;
    }

    const auto& jnt_name = item.claimed_resources[0].resources[0];
    const auto topic = item.name + "/command";
    ctrl_map_[jnt_name] = make_pair(cmd_type, nh_.advertise<std_msgs::Float64>(topic, 1));
  }

  return 0;
}

void JointCommandHandler::jointPositionsCmdCb(const tobas_msgs::JointCommandArrayConstPtr& positions)
{
  if (ctrl_map_.size() == 0 && initialize() < 0)
  {
    ctrl_map_.clear();
    return;
  }

  for (size_t i = 0; i < positions->commands.size(); ++i)
  {
    const auto& jnt_name = positions->commands[i].name;
    if (!tobas_std::contains(ctrl_map_, jnt_name))
    {
      TOBAS_ERROR("Transmission for joint '", jnt_name, "' is not found.");
      continue;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == POSITION)
    {
      const auto cmd = boost::make_shared<std_msgs::Float64>();
      cmd->data = positions->commands[i].data;
      pub.publish(cmd);
    }
    else
    {
      TOBAS_WARN(
        "Transmission type for joint '", jnt_name, "' is not position. So received position command for joint '",
        jnt_name, "' is ignored.");
    }
  }
}

void JointCommandHandler::jointVelocitiesCmdCb(const tobas_msgs::JointCommandArrayConstPtr& velocities)
{
  if (ctrl_map_.size() == 0 && initialize() < 0)
  {
    ctrl_map_.clear();
    return;
  }

  for (size_t i = 0; i < velocities->commands.size(); ++i)
  {
    const auto& jnt_name = velocities->commands[i].name;

    if (!tobas_std::contains(ctrl_map_, jnt_name))
    {
      TOBAS_ERROR("Transmission for joint '", jnt_name, "' is not found.");
      continue;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == VELOCITY)
    {
      const auto cmd = boost::make_shared<std_msgs::Float64>();
      cmd->data = velocities->commands[i].data;
      pub.publish(cmd);
    }
    else
    {
      TOBAS_WARN(
        "Transmission type for joint '", jnt_name, "' is not velocity. So received velocity command for joint '",
        jnt_name, "' is ignored.");
    }
  }
}

void JointCommandHandler::jointEffortsCmdCb(const tobas_msgs::JointCommandArrayConstPtr& efforts)
{
  if (ctrl_map_.size() == 0 && initialize() < 0)
  {
    ctrl_map_.clear();
    return;
  }

  for (size_t i = 0; i < efforts->commands.size(); ++i)
  {
    const auto& jnt_name = efforts->commands[i].name;
    if (!tobas_std::contains(ctrl_map_, jnt_name))
    {
      TOBAS_ERROR("Transmission for joint '", jnt_name, "' is not found.");
      continue;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == EFFORT)
    {
      const auto cmd = boost::make_shared<std_msgs::Float64>();
      cmd->data = efforts->commands[i].data;
      pub.publish(cmd);
    }
    else
    {
      TOBAS_WARN(
        "Transmission type for joint '", jnt_name, "' is not effort. So received effort command for joint '", jnt_name,
        "' is ignored.");
    }
  }
}
}  // namespace tobas_gazebo_ros
