#include "../include/tobas_tools/node.hpp"

namespace tobas
{
BaseNode::BaseNode(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
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
