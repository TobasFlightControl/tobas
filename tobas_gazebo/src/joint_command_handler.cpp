#include <std_msgs/Float64.h>
#include <controller_manager_msgs/ListControllers.h>

#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/util.hpp>

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
  js_sub_ =
    nh_.subscribe(tobas::kJointStatesCmdTopic, 1, &self::jointStateCmdCb, this, tcpNoDelay());
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

void JointCommandHandler::jointStateCmdCb(const sensor_msgs::JointStateConstPtr& js)
{
  if (!dh_ros::isFieldSizeMatch(*js))
  {
    rosError(name_, "The sizes of each field in JointState do not match.");
    return;
  }

  if (ctrl_map_.size() == 0 && initialize() < 0)
  {
    ctrl_map_.clear();
    return;
  }

  for (size_t i = 0; i < js->name.size(); ++i)
  {
    const auto& jnt_name = js->name[i];

    if (!ctrl_map_.contains(jnt_name))
    {
      rosError(name_, "Controller for joint '" << jnt_name << "' is not found.");
      continue;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    const auto cmd = boost::make_shared<std_msgs::Float64>();

    switch (type)
    {
      case POSITION:
        cmd->data = js->position[i];
        break;
      case VELOCITY:
        cmd->data = js->velocity[i];
        break;
      case EFFORT:
        cmd->data = js->effort[i];
        break;
      default:
        rosError(name_, "Unknown command type: " << type);
        continue;
    }

    pub.publish(cmd);
  }
}
}  // namespace tobas_gazebo
