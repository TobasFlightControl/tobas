#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_common_actions/static_state_determination_server.hpp"

namespace tobas_common_actions
{
class StaticStateDeterminationServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<StaticStateDeterminationServer> node_;
};
}  // namespace tobas_common_actions
