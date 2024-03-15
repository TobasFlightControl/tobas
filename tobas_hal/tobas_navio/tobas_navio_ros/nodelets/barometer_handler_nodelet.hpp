#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_navio_ros/barometer_handler.hpp"

namespace tobas_navio_ros
{
class BarometerHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<BarometerHandler> node_;
};
}  // namespace tobas_navio_ros
