#include <pluginlib/class_list_macros.hpp>

#include "./gps_handler_nodelet.hpp"

namespace tobas_real
{
void GpsHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing GPS Handler Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new GpsHandler(nh, pnh));
}
}  // namespace tobas_real

PLUGINLIB_EXPORT_CLASS(tobas_real::GpsHandlerNodelet, nodelet::Nodelet);
