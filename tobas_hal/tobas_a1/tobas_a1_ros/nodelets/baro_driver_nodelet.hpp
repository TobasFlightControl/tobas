#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_a1_ros/baro_driver.hpp"

namespace a1
{
class BaroDriverNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<BaroDriver> node_;
};
}  // namespace a1
