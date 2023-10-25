#include <pluginlib/class_list_macros.hpp>

#include "./eskf_nodelet.hpp"

namespace state_estimation_eskf
{
void ErrorStateKalmanFilterNodelet::onInit()
{
  NODELET_INFO("Initializing Error State Kalman Filter Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new ErrorStateKalmanFilterRos(nh, pnh, name));
}
}  // namespace state_estimation_eskf

PLUGINLIB_EXPORT_CLASS(state_estimation_eskf::ErrorStateKalmanFilterNodelet, nodelet::Nodelet);
