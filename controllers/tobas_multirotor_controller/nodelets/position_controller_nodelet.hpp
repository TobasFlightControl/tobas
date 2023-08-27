#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_multirotor_controller/position_controller_ros.hpp"

namespace tobas_multirotor_controller
{
class PositionControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  PositionControllerRos node_;
};
}  // namespace tobas_multirotor_controller
