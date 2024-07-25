#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_a1_ros/gnss_driver.hpp"

namespace a1
{
class GNSSDriverNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<GNSSDriver> node_;
};
}  // namespace a1
