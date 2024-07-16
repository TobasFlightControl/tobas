#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_a1_ros/sbus_handler.hpp"

namespace tobas_a1_ros
{
class SBUSHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<SBUSHandler> node_;
};
}  // namespace tobas_a1_ros
