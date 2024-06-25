#include <pluginlib/class_list_macros.hpp>

#include "./cpu_handler_nodelet.hpp"

namespace tobas_real_ros
{
void CpuHandlerNodelet::onInit()
{
  node_.reset(new CpuHandler(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_real_ros

PLUGINLIB_EXPORT_CLASS(tobas_real_ros::CpuHandlerNodelet, nodelet::Nodelet);
