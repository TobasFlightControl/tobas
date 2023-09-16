#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_mr_pidmpc/position_controller_ros.hpp"

namespace tobas_mr_pidmpc
{
class PositionControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<PositionControllerRos> node_;
};
}  // namespace tobas_mr_pidmpc
