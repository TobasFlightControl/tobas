#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_real_ros/magnetometer_handler.hpp"

namespace tobas_real_ros
{
class MagnetometerHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<MagnetometerHandler> node_;
};
}  // namespace tobas_real_ros
