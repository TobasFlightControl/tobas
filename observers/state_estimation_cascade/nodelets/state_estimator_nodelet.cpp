#include <pluginlib/class_list_macros.hpp>

#include "./state_estimator_nodelet.hpp"

namespace state_estimation_cascade
{
void StateEstimatorNodelet::onInit()
{
  NODELET_INFO("Initializing State Estimator Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new StateEstimator(nh, pnh, name));
}
}  // namespace state_estimation_cascade

PLUGINLIB_EXPORT_CLASS(state_estimation_cascade::StateEstimatorNodelet, nodelet::Nodelet);
