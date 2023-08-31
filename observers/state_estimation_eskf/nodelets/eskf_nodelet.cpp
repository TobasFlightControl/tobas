#include <pluginlib/class_list_macros.hpp>

#include "./eskf_nodelet.hpp"

namespace state_estimation_eskf
{
void ErrorStateKalmanFilterNodelet::onInit()
{
  NODELET_INFO("Initializing Error State Kalman Filter Nodelet.");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  node_.reset(new ErrorStateKalmanFilterRos(nh, pnh));
}
}  // namespace state_estimation_eskf

PLUGINLIB_EXPORT_CLASS(state_estimation_eskf::ErrorStateKalmanFilterNodelet, nodelet::Nodelet);
