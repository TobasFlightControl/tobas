#include <pluginlib/class_list_macros.hpp>

#include "./state_estimator_nodelet.hpp"

namespace state_estimation_cascade
{
void StateEstimatorNodelet::onInit()
{
  NODELET_INFO("Initializing State Estimator Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new StateEstimator(nh, pnh));
}
}  // namespace state_estimation_cascade

PLUGINLIB_EXPORT_CLASS(state_estimation_cascade::StateEstimatorNodelet, nodelet::Nodelet);
