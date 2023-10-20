#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_arducopter_takeoff/takeoff_action_server.hpp"

namespace tobas_arducopter_takeoff
{
class TakeoffActionServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<TakeoffActionServer> node_;
};
}  // namespace tobas_arducopter_takeoff
