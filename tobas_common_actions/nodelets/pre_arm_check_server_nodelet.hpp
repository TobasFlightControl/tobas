#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_common_actions/pre_arm_check_server.hpp"

namespace tobas_common_actions
{
class PreArmCheckServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<PreArmCheckServer> node_;
};
}  // namespace tobas_common_actions
