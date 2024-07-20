#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_a1_ros/imu_handler.hpp"

namespace a1
{
class IMUHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<IMUHandler> node_;
};
}  // namespace a1
