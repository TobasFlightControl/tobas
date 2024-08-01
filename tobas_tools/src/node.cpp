#include "../include/tobas_tools/node.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;

namespace tobas
{
BaseNode::BaseNode(, const string& name) : node_(node), pnh_(pnh), name_(name)
{
  message_pub_ = node_.advertise<tobas_msgs::Message>(tobas::kMessageTopic, 1);
}

rclcpp::TransportHints BaseNode::tcpNoDelay(const bool& nodelay)
{
  return rclcpp::TransportHints().tcpNoDelay(nodelay);
}
}  // namespace tobas
