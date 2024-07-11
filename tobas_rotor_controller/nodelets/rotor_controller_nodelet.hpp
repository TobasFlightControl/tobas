#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_rotor_controller/rotor_controller.hpp"

namespace tobas_rotor_controller
{
class RotorControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<RotorController> node_;
};
}  // namespace tobas_rotor_controller
