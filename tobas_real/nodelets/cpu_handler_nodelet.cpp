#include <pluginlib/class_list_macros.hpp>

#include "./cpu_handler_nodelet.hpp"

namespace tobas_real
{
void CpuHandlerNodelet::onInit()
{
  NODELET_INFO("Initializing CPU Handler Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new CpuHandler(nh, pnh, name));
}
}  // namespace tobas_real

PLUGINLIB_EXPORT_CLASS(tobas_real::CpuHandlerNodelet, nodelet::Nodelet);
