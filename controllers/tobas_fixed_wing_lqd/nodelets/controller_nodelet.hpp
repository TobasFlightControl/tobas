#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_fixed_wing_lqd/controller.hpp"

namespace tobas_fixed_wing_lqd
{
class ControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<Controller> node_;
};
}  // namespace tobas_fixed_wing_lqd
