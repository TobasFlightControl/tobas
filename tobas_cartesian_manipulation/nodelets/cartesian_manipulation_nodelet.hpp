#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_cartesian_manipulation/cartesian_manipulation_ros.hpp"

namespace tobas_cartesian_manipulation
{
class CartesianManipulationNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<CartesianManipulationRos> node_;
};
}  // namespace tobas_cartesian_manipulation
