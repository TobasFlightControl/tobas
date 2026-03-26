#include <tobas_constants/ros_interface.hpp>
#include <tobas_dynamixel_ros_interface/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_dynamixel_msgs/msg/motor_command_array.hpp>
#include <tobas_dynamixel_msgs/msg/motor_state_array.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>

namespace tobas
{
namespace dxl
{
/* tobas_msgsとtobas_dynamixel_msgsのブリッジ． */
class DynamixelBridgeNode : public tobas::BaseNode
{
  using self = DynamixelBridgeNode;
  using super = tobas::BaseNode;

public:
  explicit DynamixelBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Parameters
  std::unordered_set<std::string> jnt_names_;

  ros2::PublisherPtr<tobas_msgs::msg::JointStateArray> joint_states_pub_;
  ros2::SubscriberPtr<tobas_dynamixel_msgs::msg::MotorStateArray> motor_states_sub_;

  ros2::PublisherPtr<tobas_dynamixel_msgs::msg::MotorCommandArray> motor_pos_pub_;
  ros2::PublisherPtr<tobas_dynamixel_msgs::msg::MotorCommandArray> motor_vel_pub_;
  ros2::PublisherPtr<tobas_dynamixel_msgs::msg::MotorCommandArray> motor_eff_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> joint_pos_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> joint_vel_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> joint_eff_sub_;

  tobas_dynamixel_msgs::msg::MotorCommandArray::UniquePtr
  createMotorCommands(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& commands_in);

  void motorStatesCb(const tobas_dynamixel_msgs::msg::MotorStateArray::ConstSharedPtr& states_in);
  void jointPosCommandsCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& commands_in);
  void jointVelCommandsCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& commands_in);
  void jointEffCommandsCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& commands_in);
};

DynamixelBridgeNode::DynamixelBridgeNode(const rclcpp::NodeOptions& options)
  : super("dynamixel_bridge", nodeOptions_Default(options))
{
  const auto jnt_names = getStringArrayParam("joint_names", {});
  if (jnt_names.empty()) {
    TOBAS_ERROR("Joint names are not specified.");
    return;
  }
  jnt_names_.insert(jnt_names.begin(), jnt_names.end());

  joint_states_pub_ = createPublisher<tobas_msgs::msg::JointStateArray>(tobas::topic::kJointStates);
  motor_states_sub_ = createSubscriber(topic::kMotorStates, &self::motorStatesCb, this);

  motor_pos_pub_ = createPublisher<tobas_dynamixel_msgs::msg::MotorCommandArray>(topic::kPositionCommand);
  motor_vel_pub_ = createPublisher<tobas_dynamixel_msgs::msg::MotorCommandArray>(topic::kVelocityCommand);
  motor_eff_pub_ = createPublisher<tobas_dynamixel_msgs::msg::MotorCommandArray>(topic::kEffortCommand);
  joint_pos_sub_ = createSubscriber(tobas::topic::kJointPosCmd, &self::jointPosCommandsCb, this);
  joint_vel_sub_ = createSubscriber(tobas::topic::kJointVelCmd, &self::jointVelCommandsCb, this);
  joint_eff_sub_ = createSubscriber(tobas::topic::kJointEffCmd, &self::jointEffCommandsCb, this);
}

tobas_dynamixel_msgs::msg::MotorCommandArray::UniquePtr
DynamixelBridgeNode::createMotorCommands(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& commands_in)
{
  auto commands_out = std::make_unique<tobas_dynamixel_msgs::msg::MotorCommandArray>();
  commands_out->header = commands_in->header;

  for (const auto& command_in : commands_in->commands) {
    if (!jnt_names_.contains(command_in.name)) {
      continue;
    }
    commands_out->commands.emplace_back();
    commands_out->commands.back().name = command_in.name;
    commands_out->commands.back().data = command_in.data;
  }

  return commands_out;
}

void DynamixelBridgeNode::motorStatesCb(const tobas_dynamixel_msgs::msg::MotorStateArray::ConstSharedPtr& states_in)
{
  auto states_out = std::make_unique<tobas_msgs::msg::JointStateArray>();
  states_out->header = states_in->header;

  for (const auto& state_in : states_in->states) {
    if (!jnt_names_.contains(state_in.name)) {
      continue;
    }
    states_out->states.emplace_back();
    states_out->states.back().name = state_in.name;
    states_out->states.back().position = state_in.position;
    states_out->states.back().velocity = state_in.velocity;
    states_out->states.back().effort = NAN;  // TODO: MotorStateにトルク定数から求めたトルクを含める
  }

  if (!states_out->states.empty()) {
    joint_states_pub_->publish(std::move(states_out));
  }
}

void DynamixelBridgeNode::jointPosCommandsCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& commands_in)
{
  auto commands_out = createMotorCommands(commands_in);
  if (!commands_out->commands.empty()) {
    motor_pos_pub_->publish(std::move(commands_out));
  }
}

void DynamixelBridgeNode::jointVelCommandsCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& commands_in)
{
  auto commands_out = createMotorCommands(commands_in);
  if (!commands_out->commands.empty()) {
    motor_vel_pub_->publish(std::move(commands_out));
  }
}

void DynamixelBridgeNode::jointEffCommandsCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& commands_in)
{
  auto commands_out = createMotorCommands(commands_in);
  if (!commands_out->commands.empty()) {
    motor_eff_pub_->publish(std::move(commands_out));
  }
}
}  // namespace dxl
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::dxl::DynamixelBridgeNode)
