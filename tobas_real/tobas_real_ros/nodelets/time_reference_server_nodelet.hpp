#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_real_ros/time_reference_server.hpp"

namespace tobas_real
{
class TimeReferenceServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<TimeReferenceServer> node_;
};
}  // namespace tobas_real
