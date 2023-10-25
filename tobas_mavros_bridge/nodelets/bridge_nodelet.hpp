#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_mavros_bridge/bridge.hpp"

namespace tobas_mavros_bridge
{
class TobasMavrosBridgeNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<TobasMavrosBridge> node_;
};
}  // namespace tobas_mavros_bridge
