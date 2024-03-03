#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_manipulation/position_controller_ros.hpp"

namespace tobas_manipulation
{
class PositionControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<PositionControllerRos> node_;
};
}  // namespace tobas_manipulation
