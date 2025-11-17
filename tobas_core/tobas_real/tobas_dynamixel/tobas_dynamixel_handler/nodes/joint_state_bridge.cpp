#include <tobas_constants/constants.hpp>
#include <tobas_dynamixel_ros_interface/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_dynamixel_msgs/msg/motor_state_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>

namespace tobas_dynamixel
{
class JointStateBridgeNode : public tobas::BaseNode
{
  using self = JointStateBridgeNode;
  using super = tobas::BaseNode;

public:
  explicit JointStateBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<tobas_msgs::msg::JointStateArray> joint_states_pub_;
  ros2::SubscriberPtr<tobas_dynamixel_msgs::msg::MotorStateArray> motor_states_sub_;

  void motorStatesCb(const tobas_dynamixel_msgs::msg::MotorStateArray::ConstSharedPtr& motor_states);
};

JointStateBridgeNode::JointStateBridgeNode(const rclcpp::NodeOptions& options) : super("dynamixel_joint_state_bridge", options)
{
  joint_states_pub_ = createPublisher<tobas_msgs::msg::JointStateArray>(tobas::kJointStatesTopic);
  motor_states_sub_ = createSubscriber(topic::kMotorStates, &self::motorStatesCb, this);
}

void JointStateBridgeNode::motorStatesCb(const tobas_dynamixel_msgs::msg::MotorStateArray::ConstSharedPtr& motor_states)
{
  auto joint_states = std::make_unique<tobas_msgs::msg::JointStateArray>();
  joint_states->header = motor_states->header;

  for (const auto& motor_state : motor_states->states) {
    joint_states->states.emplace_back();
    joint_states->states.back().name = motor_state.name;
    joint_states->states.back().position = motor_state.position;
    joint_states->states.back().velocity = motor_state.velocity;
    joint_states->states.back().effort = NAN;  // TODO: MotorStateにトルク定数から求めたトルクを含める
  }

  joint_states_pub_->publish(std::move(joint_states));
}
}  // namespace tobas_dynamixel

RCLCPP_COMPONENTS_REGISTER_NODE(tobas_dynamixel::JointStateBridgeNode)
