#include <std_msgs/msg/float64_multi_array.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

#include <tobas_std_tools/string.hpp>
#include <tobas_ros2_tools/simple_service_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_core/joint_interface.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>

using namespace std;
using namespace std_msgs::msg;

/**
 * @brief ジョイントの位置，速度，力のコマンドを受け取り，Gazeboのトランスミッションに指令する．
 */
class JointCommandHandlerNode : public tobas::BaseNode
{
  using self = JointCommandHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit JointCommandHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  unordered_map<string, pair<tobas::joint_interface_t, ros2::PublisherPtr<Float64MultiArray>>> ctrl_map_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> positions_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> velocities_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> efforts_sub_;

  bool initialize();

  void jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions);
  void jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities);
  void jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts);
};

JointCommandHandlerNode::JointCommandHandlerNode(const rclcpp::NodeOptions& options)
  : super("gazebo_joint_command_handler", options)
{
  positions_sub_ = createSubscriber(tobas::kJointPositionsCmdTopic, &self::jointPositionsCmdCb, this);
  velocities_sub_ = createSubscriber(tobas::kJointVelocitiesCmdTopic, &self::jointVelocitiesCmdCb, this);
  efforts_sub_ = createSubscriber(tobas::kJointEffortsCmdTopic, &self::jointEffortsCmdCb, this);
}

bool JointCommandHandlerNode::initialize()
{
  // ノードの起動順が不確定なため，サービスコールをコンストラクタでやるべきではない
  ros2::SimpleServiceClient<controller_manager_msgs::srv::ListControllers> sc(
    shared_from_this(), tobas::kListControllersSrv);

  const auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  if (!sc.call(req))
  {
    TOBAS_ERROR("Failed to call \"", tobas::kListControllersSrv, "\" service.");
    return false;
  }

  for (const auto& item : sc.getResponse()->controller)
  {
    if (item.claimed_interfaces.size() == 0)
    {
      TOBAS_WARN("No joints are registered to \"", item.name, "\".");
      continue;
    }
    else if (item.claimed_interfaces.size() >= 2)
    {
      TOBAS_WARN("Controllers that handle multiple joints are not supported.");
      continue;
    }

    // TODO: item.typeによる場合分けは必要？

    const auto& [jnt_name, jnt_cmd_if] = tobas_std::rsplit(item.claimed_interfaces.at(0), '/');
    const auto interface = tobas::jointIFTextToEnum(jnt_cmd_if);

    const auto topic = item.name + "/commands";
    ctrl_map_[jnt_name] = make_pair(interface, createPublisher<Float64MultiArray>(topic, 1));
  }

  return true;
}

void JointCommandHandlerNode::jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions)
{
  if (ctrl_map_.size() == 0)
  {
    if (!initialize())
    {
      ctrl_map_.clear();
      return;
    }
  }

  for (size_t i = 0; i < positions->commands.size(); ++i)
  {
    const auto& jnt_name = positions->commands[i].name;
    if (!ctrl_map_.contains(jnt_name))
    {
      TOBAS_ERROR("Controller for joint '", jnt_name, "' is not found.");
      return;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == tobas::joint_interface_t::POSITION)
    {
      auto cmd = std::make_unique<Float64MultiArray>();
      cmd->data.push_back(positions->commands[i].data);
      pub->publish(move(cmd));
    }
    else
    {
      TOBAS_WARN(
        "Controller type for joint '", jnt_name, "' is not position. So received position command for joint '",
        jnt_name, "' is ignored.");
    }
  }
}

void JointCommandHandlerNode::jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities)
{
  if (ctrl_map_.size() == 0)
  {
    if (!initialize())
    {
      ctrl_map_.clear();
      return;
    }
  }

  for (size_t i = 0; i < velocities->commands.size(); ++i)
  {
    const auto& jnt_name = velocities->commands[i].name;

    if (!ctrl_map_.contains(jnt_name))
    {
      TOBAS_ERROR("Controller for joint '", jnt_name, "' is not found.");
      return;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == tobas::joint_interface_t::VELOCITY)
    {
      auto cmd = std::make_unique<Float64MultiArray>();
      cmd->data.push_back(velocities->commands[i].data);
      pub->publish(move(cmd));
    }
    else
    {
      TOBAS_WARN(
        "Controller type for joint '", jnt_name, "' is not velocity. So received velocity command for joint '",
        jnt_name, "' is ignored.");
    }
  }
}

void JointCommandHandlerNode::jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts)
{
  if (ctrl_map_.size() == 0)
  {
    if (!initialize())
    {
      ctrl_map_.clear();
      return;
    }
  }

  for (size_t i = 0; i < efforts->commands.size(); ++i)
  {
    const auto& jnt_name = efforts->commands[i].name;
    if (!ctrl_map_.contains(jnt_name))
    {
      TOBAS_ERROR("Controller for joint '", jnt_name, "' is not found.");
      return;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == tobas::joint_interface_t::EFFORT)
    {
      auto cmd = std::make_unique<Float64MultiArray>();
      cmd->data.push_back(efforts->commands[i].data);
      pub->publish(move(cmd));
    }
    else
    {
      TOBAS_WARN(
        "Controller type for joint '", jnt_name, "' is not effort. So received effort command for joint '", jnt_name,
        "' is ignored.");
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(JointCommandHandlerNode)
