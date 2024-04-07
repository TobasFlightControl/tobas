#include <pluginlib/class_list_macros.hpp>

#include "./latency_publisher_nodelet.hpp"

namespace tobas_latency_publisher
{
void LatencyPublisherNodelet::onInit()
{
  NODELET_INFO("Initializing Battery LPF Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new LatencyPublisher(nh, pnh, name));
}
}  // namespace tobas_latency_publisher

PLUGINLIB_EXPORT_CLASS(tobas_latency_publisher::LatencyPublisherNodelet, nodelet::Nodelet);
