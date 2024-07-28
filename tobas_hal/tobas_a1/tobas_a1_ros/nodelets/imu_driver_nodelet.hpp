#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_a1_ros/imu_driver.hpp"

namespace a1
{
class IMUDriverNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<IMUDriver> node_;
};
}  // namespace a1
