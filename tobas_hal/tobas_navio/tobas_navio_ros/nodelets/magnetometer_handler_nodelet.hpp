#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_navio_ros/magnetometer_handler.hpp"

namespace tobas_navio_ros
{
class MagnetometerHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<MagnetometerHandler> node_;
};
}  // namespace tobas_navio_ros
