#include <pluginlib/class_list_macros.hpp>

#include "./move_action_server_nodelet.hpp"

namespace tobas_multirotor_move
{
void MoveActionServerNodelet::onInit()
{
  node_.reset(new MoveActionServer(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_multirotor_move

PLUGINLIB_EXPORT_CLASS(tobas_multirotor_move::MoveActionServerNodelet, nodelet::Nodelet);
