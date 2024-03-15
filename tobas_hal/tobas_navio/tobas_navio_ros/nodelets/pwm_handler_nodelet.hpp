#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_navio_ros/pwm_handler.hpp"

namespace tobas_navio_ros
{
class PwmHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<PwmHandler> node_;
};
}  // namespace tobas_navio_ros
