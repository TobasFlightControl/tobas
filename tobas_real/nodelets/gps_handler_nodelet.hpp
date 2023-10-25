#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_real/gps_handler.hpp"

namespace tobas_real
{
class GpsHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<GpsHandler> node_;
};
}  // namespace tobas_real
