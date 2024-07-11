#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_real_ros/battery_handler.hpp"

namespace tobas_real_ros
{
class BatteryHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<BatteryHandler> node_;
};
}  // namespace tobas_real_ros
