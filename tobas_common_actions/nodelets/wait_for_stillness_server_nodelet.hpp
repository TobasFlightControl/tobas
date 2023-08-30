#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_common_actions/wait_for_stillness_server.hpp"

namespace tobas_common_actions
{
class WaitForStillnessServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<WaitForStillnessServer> node_;
};
}  // namespace tobas_common_actions
