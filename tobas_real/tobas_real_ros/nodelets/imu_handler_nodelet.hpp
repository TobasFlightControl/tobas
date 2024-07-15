#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_real_ros/imu_handler.hpp"

namespace tobas_real_ros
{
class ImuHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<ImuHandler> node_;
};
}  // namespace tobas_real_ros
