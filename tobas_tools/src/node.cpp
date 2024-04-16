#include <tobas_ros_tools/console_message.hpp>

#include "../include/tobas_tools/node.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;

namespace tobas
{
BaseNode::BaseNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : nh_(nh), pnh_(pnh), name_(name)
{
  message_pub_ = nh_.advertise<tobas_msgs::Message>(tobas::kMessageTopic, 1);
}

ros::TransportHints BaseNode::tcpNoDelay(const bool& nodelay)
{
  return ros::TransportHints().reliable().tcpNoDelay(nodelay);
}
}  // namespace tobas
