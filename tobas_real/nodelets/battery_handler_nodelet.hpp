#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_real/battery_handler.hpp"

namespace tobas_real
{
class BatteryHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<BatteryHandler> node_;
};
}  // namespace tobas_real
