#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_a1_ros/sbus_driver.hpp"

namespace tobas_a1_ros
{
class SBUSDriverNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<SBUSDriver> node_;
};
}  // namespace tobas_a1_ros
