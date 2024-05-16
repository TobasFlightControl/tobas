#include <pluginlib/class_list_macros.hpp>

#include "./move_action_server_nodelet.hpp"

namespace tobas_multirotor_move
{
void MoveActionServerNodelet::onInit()
{
  NODELET_INFO("Initializing Multirotor Move Action Server Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new MoveActionServer(nh, pnh, name));
}
}  // namespace tobas_multirotor_move

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_move::MoveActionServerNodelet, nodelet::Nodelet);
