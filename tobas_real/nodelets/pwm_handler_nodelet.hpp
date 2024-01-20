#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_real/pwm_handler.hpp"

namespace tobas_real
{
class PwmHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<PwmHandler> node_;
};
}  // namespace tobas_real
