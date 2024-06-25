#include <pluginlib/class_list_macros.hpp>

#include "./magnetometer_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void MagnetometerHandlerNodelet::onInit()
{
  node_.reset(new MagnetometerHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::MagnetometerHandlerNodelet, nodelet::Nodelet);
