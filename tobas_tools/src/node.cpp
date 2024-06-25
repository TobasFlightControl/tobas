#include "../include/tobas_tools/node.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;

namespace tobas
{
BaseNode::BaseNode(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name) : nh_(nh), pnh_(pnh), name_(name)
{
  message_pub_ = nh_.advertise<tobas_msgs::Message>(tobas::kMessageTopic, 1);
}

ros::TransportHints BaseNode::tcpNoDelay(const bool& nodelay)
{
  return ros::TransportHints().tcpNoDelay(nodelay);
}
}  // namespace tobas
