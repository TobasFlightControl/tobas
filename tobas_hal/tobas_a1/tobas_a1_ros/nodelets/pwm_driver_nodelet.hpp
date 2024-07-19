#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_a1_ros/pwm_driver.hpp"

namespace a1
{
class PWMDriverNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<PWMDriver> node_;
};
}  // namespace a1
