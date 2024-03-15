#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_navio_ros/rcin_handler.hpp"

namespace tobas_navio_ros
{
class RCInputHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<RCInputHandler> node_;
};
}  // namespace tobas_navio_ros
