#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_multirotor_controller/controller_ros.hpp"

namespace tobas_multirotor_controller
{
class ControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<ControllerRos> node_;
};
}  // namespace tobas_multirotor_controller
