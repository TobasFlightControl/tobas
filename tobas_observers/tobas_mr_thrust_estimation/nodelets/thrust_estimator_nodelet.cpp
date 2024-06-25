#include <pluginlib/class_list_macros.hpp>

#include "./thrust_estimator_nodelet.hpp"

namespace tobas_mr_thrust_estimation
{
void ThrustEstimatorNodelet::onInit()
{
  node_.reset(new ThrustEstimator(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_mr_thrust_estimation

PLUGINLIB_EXPORT_CLASS(tobas_mr_thrust_estimation::ThrustEstimatorNodelet, nodelet::Nodelet);
