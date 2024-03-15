#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_navio_ros/cpu_handler.hpp"

namespace tobas_navio_ros
{
class CpuHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<CpuHandler> node_;
};
}  // namespace tobas_navio_ros
