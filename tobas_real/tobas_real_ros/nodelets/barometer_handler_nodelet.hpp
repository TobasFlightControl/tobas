#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_real_ros/barometer_handler.hpp"

namespace tobas_real_ros
{
class BarometerHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<BarometerHandler> node_;
};
}  // namespace tobas_real_ros
