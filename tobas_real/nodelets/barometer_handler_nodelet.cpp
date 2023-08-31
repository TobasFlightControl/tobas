#include <pluginlib/class_list_macros.hpp>

#include "./barometer_handler_nodelet.hpp"

namespace tobas_real
{
void BarometerHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing Barometer Handler Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new BarometerHandler(nh, pnh));
}
}  // namespace tobas_real

PLUGINLIB_EXPORT_CLASS(tobas_real::BarometerHandlerNodelet, nodelet::Nodelet);
