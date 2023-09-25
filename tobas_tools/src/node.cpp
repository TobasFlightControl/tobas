#include "../include/tobas_tools/node.hpp"

namespace tobas
{
BaseNode::BaseNode(ros::NodeHandle nh, ros::NodeHandle pnh, const std::string& name)
  : nh_(nh), pnh_(pnh), name_(name)
{
}

ros::TransportHints BaseNode::tcpNoDelay(const bool& nodelay)
{
  return ros::TransportHints().reliable().tcpNoDelay(nodelay);
}
}  // namespace tobas
