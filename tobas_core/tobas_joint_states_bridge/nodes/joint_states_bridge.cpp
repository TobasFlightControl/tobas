// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <tobas_msgs/msg/joint_state_array.hpp>

namespace tobas
{
/* tobas_msgs/JointStateArray -> sensor_msgs/JointState */
class JointStatesBridgeNode : public BaseNode
{
  using self = JointStatesBridgeNode;
  using super = BaseNode;

public:
  explicit JointStatesBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<sensor_msgs::msg::JointState> js_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> js_sub_;

  void jointStatesCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js_in);
};

JointStatesBridgeNode::JointStatesBridgeNode(const rclcpp::NodeOptions& options)
  : super("joint_states_bridge", nodeOptions_Default(options))
{
  js_pub_ = createPublisher<sensor_msgs::msg::JointState>("joint_states");
  js_sub_ = createSubscriber<tobas_msgs::msg::JointStateArray>(topic::kJointStates, &self::jointStatesCb, this);
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
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::JointStatesBridgeNode)
