#include <tobas_constants/constants.hpp>
#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <tobas_msgs/msg/joint_state_array.hpp>

/* tobas_msgs/JointStateArray -> sensor_msgs/JointState */
class JointStatesBridgeNode : public tobas::BaseNode
{
  using self = JointStatesBridgeNode;
  using super = tobas::BaseNode;

public:
  explicit JointStatesBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<sensor_msgs::msg::JointState> js_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> js_sub_;

  void jointStatesCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js_in);
};

JointStatesBridgeNode::JointStatesBridgeNode(const rclcpp::NodeOptions& options) : super("joint_states_bridge", options)
{
  js_pub_ = createPublisher<sensor_msgs::msg::JointState>("joint_states");
  js_sub_ = createSubscriber<tobas_msgs::msg::JointStateArray>(tobas::kJointStatesTopic, &self::jointStatesCb, this);
}

void JointStatesBridgeNode::jointStatesCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js_in)
{
  auto js_out = std::make_unique<sensor_msgs::msg::JointState>();
  js_out->header = js_in->header;

  for (const auto& state : js_in->states) {
    js_out->name.push_back(state.name);
    js_out->position.push_back(state.position);
    js_out->velocity.push_back(state.velocity);
    js_out->effort.push_back(state.effort);
  }

  js_pub_->publish(std::move(js_out));
}

RCLCPP_COMPONENTS_REGISTER_NODE(JointStatesBridgeNode)
