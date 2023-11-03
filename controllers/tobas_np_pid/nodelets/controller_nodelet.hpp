#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_np_pid/controller_ros.hpp"

namespace tobas_np_pid
{
class ControllerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<ControllerRos> node_;
};
}  // namespace tobas_np_pid
