#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_mr_arducopter/controller_ros.hpp"

namespace tobas_mr_arducopter
{
class ControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<ControllerRos> node_;
};
}  // namespace tobas_mr_arducopter
