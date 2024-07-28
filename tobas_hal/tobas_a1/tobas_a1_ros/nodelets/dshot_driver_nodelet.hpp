#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_a1_ros/dshot_driver.hpp"

namespace a1
{
class DShotDriverNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<DShotDriver> node_;
};
}  // namespace a1
