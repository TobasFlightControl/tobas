#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_real/motors_handler.hpp"

namespace tobas_real
{
class MotorsHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<MotorsHandler> node_;
};
}  // namespace tobas_real
