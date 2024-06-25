#include <pluginlib/class_list_macros.hpp>

#include "./barometer_handler_nodelet.hpp"

namespace tobas_navio_ros
{
void BarometerHandlerNodelet::onInit()
{
  node_.reset(new BarometerHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_navio_ros

PLUGINLIB_EXPORT_CLASS(tobas_navio_ros::BarometerHandlerNodelet, nodelet::Nodelet);
