#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_pre_arm_check/pre_arm_check_server.hpp"

namespace tobas_pre_arm_check
{
class PreArmCheckServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<PreArmCheckServer> node_;
};
}  // namespace tobas_pre_arm_check
