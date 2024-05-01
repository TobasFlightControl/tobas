#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_multirotor_move/move_action_server.hpp"

namespace tobas_multirotor_move
{
class MultirotorMoveServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<MultirotorMoveServer> node_;
};
}  // namespace tobas_multirotor_move
