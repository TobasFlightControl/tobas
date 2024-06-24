#include <pluginlib/class_list_macros.hpp>

#include "./latency_publisher_nodelet.hpp"

namespace tobas_latency_publisher
{
void LatencyPublisherNodelet::onInit()
{
  node_.reset(new LatencyPublisher(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_latency_publisher

PLUGINLIB_EXPORT_CLASS(tobas_latency_publisher::LatencyPublisherNodelet, nodelet::Nodelet);
