#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_navio_ros/motors_handler.hpp"

namespace tobas_navio_ros
{
class MotorsHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<MotorsHandler> node_;
};
}  // namespace tobas_navio_ros
