#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_dynamixel_handler/dynamixel_handler.hpp"

namespace tobas_dynamixel_handler
{
class DynamixelHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<DynamixelHandler> node_;
};
}  // namespace tobas_dynamixel_handler
