#include "../include/tobas_tools/node.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;

namespace tobas
{
BaseNode::BaseNode(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr pnh, const string& name) : nh_(node), pnh_(pnh), name_(name)
{
  message_pub_ = nh_.advertise<tobas_msgs::Message>(tobas::kMessageTopic, 1);
}

rclcpp::TransportHints BaseNode::tcpNoDelay(const bool& nodelay)
{
  return rclcpp::TransportHints().tcpNoDelay(nodelay);
}
}  // namespace tobas
