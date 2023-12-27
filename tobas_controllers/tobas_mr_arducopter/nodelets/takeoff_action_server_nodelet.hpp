#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_mr_arducopter/takeoff_action_server.hpp"

namespace tobas_mr_arducopter
{
class TakeoffActionServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<TakeoffActionServer> node_;
};
}  // namespace tobas_mr_arducopter
