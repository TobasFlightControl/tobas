#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_multirotor_controller/rotation_controller_ros.hpp"

namespace tobas_multirotor_controller
{
class RotationControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<RotationControllerRos> node_;
};
}  // namespace tobas_multirotor_controller
