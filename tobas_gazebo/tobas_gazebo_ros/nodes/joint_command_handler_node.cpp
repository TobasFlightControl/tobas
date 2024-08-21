#include <std_msgs/msg/float64.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

#include <tobas_ros2_tools/simple_service_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
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
  enum command_type_t : int
  {
    POSITION,
    VELOCITY,
    EFFORT,
  };

  std::unordered_map<std::string, std::pair<command_type_t, ros2::PublisherPtr<std_msgs::msg::Float64>>> ctrl_map_;
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
  // 制限時間を設けて成功するまで何度も繰り返すのが正しい
  ros2::SimpleServiceClient<controller_manager_msgs::srv::ListControllers> sc(
    shared_from_this(), tobas::kListControllersSrv);

  const auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  if (!sc.call(req))
  {
    TOBAS_ERROR("Failed to call \"", tobas::kListControllersSrv, "\" service.");
    return false;
  }

  command_type_t control_type;
  for (const auto& item : sc.getResponse()->controller)
  {
    if (item.claimed_interfaces.size() != 1)
      continue;

    if (item.type.ends_with("JointPositionController"))
      control_type = POSITION;
    else if (item.type.ends_with("JointVelocityController"))
      control_type = VELOCITY;
    else if (item.type.ends_with("JointEffortController"))
      control_type = EFFORT;
    else
    {
      TOBAS_ERROR("Unknown controller type: ", item.type);
      return false;
    }

    const auto& jnt_name = item.claimed_interfaces.at(0);
    const auto topic = item.name + "/command";
    ctrl_map_[jnt_name] = make_pair(control_type, createPublisher<std_msgs::msg::Float64>(topic, 1));
  }

  return true;
}

void JointCommandHandlerNode::jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions)
{
  if (ctrl_map_.size() == 0 && !initialize())
  {
    ctrl_map_.clear();
    return;
  }

  for (size_t i = 0; i < positions->commands.size(); ++i)
  {
    const auto& jnt_name = positions->commands[i].name;
    if (!ctrl_map_.contains(jnt_name))
    {
      TOBAS_ERROR("Transmission for joint '", jnt_name, "' is not found.");
      continue;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == POSITION)
    {
      auto cmd = std::make_unique<std_msgs::msg::Float64>();
      cmd->data = positions->commands[i].data;
      pub->publish(move(cmd));
    }
    else
    {
      TOBAS_WARN(
        "Transmission type for joint '", jnt_name, "' is not position. So received position command for joint '",
        jnt_name, "' is ignored.");
    }
  }
}

void JointCommandHandlerNode::jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities)
{
  if (ctrl_map_.size() == 0 && !initialize())
  {
    ctrl_map_.clear();
    return;
  }

  for (size_t i = 0; i < velocities->commands.size(); ++i)
  {
    const auto& jnt_name = velocities->commands[i].name;

    if (!ctrl_map_.contains(jnt_name))
    {
      TOBAS_ERROR("Transmission for joint '", jnt_name, "' is not found.");
      continue;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == VELOCITY)
    {
      auto cmd = std::make_unique<std_msgs::msg::Float64>();
      cmd->data = velocities->commands[i].data;
      pub->publish(move(cmd));
    }
    else
    {
      TOBAS_WARN(
        "Transmission type for joint '", jnt_name, "' is not velocity. So received velocity command for joint '",
        jnt_name, "' is ignored.");
    }
  }
}

void JointCommandHandlerNode::jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts)
{
  if (ctrl_map_.size() == 0 && !initialize())
  {
    ctrl_map_.clear();
    return;
  }

  for (size_t i = 0; i < efforts->commands.size(); ++i)
  {
    const auto& jnt_name = efforts->commands[i].name;
    if (!ctrl_map_.contains(jnt_name))
    {
      TOBAS_ERROR("Transmission for joint '", jnt_name, "' is not found.");
      continue;
    }

    const auto& [type, pub] = ctrl_map_[jnt_name];
    if (type == EFFORT)
    {
      auto cmd = std::make_unique<std_msgs::msg::Float64>();
      cmd->data = efforts->commands[i].data;
      pub->publish(move(cmd));
    }
    else
    {
      TOBAS_WARN(
        "Transmission type for joint '", jnt_name, "' is not effort. So received effort command for joint '", jnt_name,
        "' is ignored.");
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(JointCommandHandlerNode)
