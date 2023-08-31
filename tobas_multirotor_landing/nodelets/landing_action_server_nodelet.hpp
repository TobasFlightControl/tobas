#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_multirotor_landing/landing_action_server.hpp"

namespace tobas_multirotor_landing
{
class MultirotorLandServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<MultirotorLandServer> node_;
};
}  // namespace tobas_multirotor_landing
