#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_multirotor_move/move_action_server.hpp"

namespace tobas_multirotor_move
{
class MoveActionServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<MoveActionServer> node_;
};
}  // namespace tobas_multirotor_move
