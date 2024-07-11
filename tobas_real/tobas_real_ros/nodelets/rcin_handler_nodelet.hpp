#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_real_ros/rcin_handler.hpp"

namespace tobas_real_ros
{
class RCInputHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<RCInputHandler> node_;
};
}  // namespace tobas_real_ros
