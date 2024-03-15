#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_navio_ros/battery_handler.hpp"

namespace tobas_navio_ros
{
class BatteryHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<BatteryHandler> node_;
};
}  // namespace tobas_navio_ros
