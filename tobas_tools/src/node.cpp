#include "../include/tobas_tools/node.hpp"

namespace tobas
{
BaseNode::BaseNode() : ns_(ros::this_node::getNamespace())
{
}

void BaseNode::requestShutdown()
{
  tobas_msgs::Event event;
  event.data = tobas_msgs::Event::SHUTDOWN;
  event_pub_.publish(event);
  ros::shutdown();  // 自身のノードも落とす
}
}  // namespace tobas
