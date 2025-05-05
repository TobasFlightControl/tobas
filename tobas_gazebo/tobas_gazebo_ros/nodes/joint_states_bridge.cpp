#include <tobas_node/node.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_constants/constants.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <tobas_msgs/msg/joint_state_array.hpp>

/* sensor_msgs/JointState -> tobas_msgs/JointStateArray */
class JointStatesBridge : public tobas::BaseNode
{
  using self = JointStatesBridge;
  using super = tobas::BaseNode;

public:
  explicit JointStatesBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<tobas_msgs::msg::JointStateArray> js_pub_;
  ros2::SubscriberPtr<sensor_msgs::msg::JointState> js_sub_;

  void jointStatesCb(const sensor_msgs::msg::JointState::ConstSharedPtr& js_in);
};

JointStatesBridge::JointStatesBridge(const rclcpp::NodeOptions& options) : super("gazebo_joint_states_bridge", options)
{
  js_pub_ = createPublisher<tobas_msgs::msg::JointStateArray>(tobas::kJointStatesTopic);
  js_sub_ = createSubscriber<sensor_msgs::msg::JointState>(gazebo::kJointStatesTopic, &self::jointStatesCb, this);
}

void JointStatesBridge::jointStatesCb(const sensor_msgs::msg::JointState::ConstSharedPtr& js_in)
{
  auto js_out = std::make_unique<tobas_msgs::msg::JointStateArray>();
  js_out->header = js_in->header;

  for (size_t i = 0; i < js_in->name.size(); ++i) {
    js_out->states.emplace_back();
    js_out->states.back().name = js_in->name[i];
    js_out->states.back().position = js_in->position[i];
    js_out->states.back().velocity = js_in->velocity[i];
    js_out->states.back().effort = js_in->effort[i];
  }

  js_pub_->publish(std::move(js_out));
}

RCLCPP_COMPONENTS_REGISTER_NODE(JointStatesBridge)
