#include <ranges>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_core/joint/command_interface.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>

using namespace std;

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
  unordered_map<string, pair<tobas::jnt_cmd_iface_t, ros2::PublisherPtr<std_msgs::msg::Float64MultiArray>>> ctrl_map_;

  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> positions_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> velocities_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> efforts_sub_;

  ros2::ServiceClientPtr<controller_manager_msgs::srv::ListControllers> list_controllers_sc_;

  void jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions);
  void jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities);
  void jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts);
};

JointCommandHandlerNode::JointCommandHandlerNode(const rclcpp::NodeOptions& options)
  : super("gazebo_joint_command_handler", options)
{
  // Register publishers
  const auto joint_names = getStringArrayParam("joint_names", {});
  const auto interfaces = getIntArrayParam("interfaces", {});
  if (joint_names.size() != interfaces.size())
    TOBAS_EXIT("The sizes of joint name array and interface array are different.");
  for (const auto& [jnt_name, iface] : views::zip(joint_names, interfaces))
  {
    const auto controller_name = jnt_name + "_controller";
    const auto topic = controller_name + "/commands";
    ctrl_map_[jnt_name] = { static_cast<tobas::jnt_cmd_iface_t>(iface),
                            createPublisher<std_msgs::msg::Float64MultiArray>(topic, false, true) };
  }

  // Register subscribers
  positions_sub_ = createSubscriber(tobas::kJointPositionsCmdTopic, &self::jointPositionsCmdCb, this);
  velocities_sub_ = createSubscriber(tobas::kJointVelocitiesCmdTopic, &self::jointVelocitiesCmdCb, this);
  efforts_sub_ = createSubscriber(tobas::kJointEffortsCmdTopic, &self::jointEffortsCmdCb, this);
}

void JointCommandHandlerNode::jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions)
{
  for (size_t i = 0; i < positions->commands.size(); ++i)
  {
    const auto& jnt_name = positions->commands[i].name;
    if (!ctrl_map_.contains(jnt_name))
    {
      TOBAS_ERROR("Controller for joint '", jnt_name, "' is not found.");
      return;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == tobas::jnt_cmd_iface_t::POSITION)
    {
      auto cmd = std::make_unique<std_msgs::msg::Float64MultiArray>();
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
  for (size_t i = 0; i < velocities->commands.size(); ++i)
  {
    const auto& jnt_name = velocities->commands[i].name;

    if (!ctrl_map_.contains(jnt_name))
    {
      TOBAS_ERROR("Controller for joint '", jnt_name, "' is not found.");
      return;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == tobas::jnt_cmd_iface_t::VELOCITY)
    {
      auto cmd = std::make_unique<std_msgs::msg::Float64MultiArray>();
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
  for (size_t i = 0; i < efforts->commands.size(); ++i)
  {
    const auto& jnt_name = efforts->commands[i].name;
    if (!ctrl_map_.contains(jnt_name))
    {
      TOBAS_ERROR("Controller for joint '", jnt_name, "' is not found.");
      return;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == tobas::jnt_cmd_iface_t::EFFORT)
    {
      auto cmd = std::make_unique<std_msgs::msg::Float64MultiArray>();
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
