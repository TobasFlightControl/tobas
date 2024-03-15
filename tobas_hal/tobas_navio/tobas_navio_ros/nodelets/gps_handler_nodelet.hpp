#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_navio_ros/gps_handler.hpp"

namespace tobas_navio_ros
{
class GpsHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<GpsHandler> node_;
};
}  // namespace tobas_navio_ros
