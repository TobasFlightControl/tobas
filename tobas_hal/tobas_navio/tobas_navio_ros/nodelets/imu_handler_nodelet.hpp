#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_navio_ros/imu_handler.hpp"

namespace tobas_navio_ros
{
class ImuHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<ImuHandler> node_;
};
}  // namespace tobas_navio_ros
