#include "../include/tobas_tools/node.hpp"

namespace tobas
{
BaseNode::BaseNode(ros::NodeHandle nh, ros::NodeHandle pnh, const std::string& name)
  : nh_(nh), pnh_(pnh), name_(name)
{
}

void BaseNode::requestShutdown()
{
  tobas_msgs::Event event;
  event.data = tobas_msgs::Event::SHUTDOWN;
  event_pub_.publish(event);
  nh_.shutdown();  // 自身のノードも落とす
}
}  // namespace tobas
