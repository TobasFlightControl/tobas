#include <pluginlib/class_list_macros.hpp>

#include "./cpu_handler_nodelet.hpp"

namespace tobas_real
{
void CpuHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing CPU Handler Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new CpuHandler(nh, pnh));
}
}  // namespace tobas_real

PLUGINLIB_EXPORT_CLASS(tobas_real::CpuHandlerNodelet, nodelet::Nodelet);
