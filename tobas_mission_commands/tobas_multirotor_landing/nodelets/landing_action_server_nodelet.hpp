#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_multirotor_landing/landing_action_server.hpp"

namespace tobas_multirotor_landing
{
class LandActionServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<LandActionServer> node_;
};
}  // namespace tobas_multirotor_landing
