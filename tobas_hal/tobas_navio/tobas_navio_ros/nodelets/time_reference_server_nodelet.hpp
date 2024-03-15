#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_navio_ros/time_reference_server.hpp"

namespace tobas_navio_ros
{
class TimeReferenceServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<TimeReferenceServer> node_;
};
}  // namespace tobas_navio_ros
