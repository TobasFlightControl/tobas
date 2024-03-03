#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_rc_teleop/rc_teleop.hpp"

namespace tobas_rc_teleop
{
class RCTeleopNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<RCTeleop> node_;
};
}  // namespace tobas_rc_teleop
