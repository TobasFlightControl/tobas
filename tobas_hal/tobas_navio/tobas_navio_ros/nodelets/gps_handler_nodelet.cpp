#include <pluginlib/class_list_macros.hpp>

#include "./gps_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void GpsHandlerNodelet::onInit()
{
  node_.reset(new GpsHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::GpsHandlerNodelet, nodelet::Nodelet);
