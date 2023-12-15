#pragma once

#include <nodelet/nodelet.h>

#include "../include/dynamixel_handler/dynamixel_handler.hpp"

namespace dynamixel_handler
{
class DynamixelHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<DynamixelHandler> node_;
};
}  // namespace dynamixel_handler
