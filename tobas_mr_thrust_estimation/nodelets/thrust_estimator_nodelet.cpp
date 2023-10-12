#include <pluginlib/class_list_macros.hpp>

#include "./thrust_estimator_nodelet.hpp"

namespace tobas_mr_thrust_estimation
{
void ThrustEstimatorNodelet::onInit()
{
  NODELET_INFO("Initializing Wind Estimator Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new ThrustEstimator(nh, pnh, name));
}
}  // namespace tobas_mr_thrust_estimation

PLUGINLIB_EXPORT_CLASS(tobas_mr_thrust_estimation::ThrustEstimatorNodelet, nodelet::Nodelet);
