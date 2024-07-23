#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_a1_ros/adc_driver.hpp"

namespace a1
{
class ADCDriverNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<ADCDriver> node_;
};
}  // namespace a1
