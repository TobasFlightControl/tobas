#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_state_checker/state_checker.hpp"

namespace tobas_state_checker
{
class StateCheckerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<StateChecker> node_;
};
}  // namespace tobas_state_checker
