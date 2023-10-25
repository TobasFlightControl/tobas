#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_real/cpu_handler.hpp"

namespace tobas_real
{
class CpuHandlerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<CpuHandler> node_;
};
}  // namespace tobas_real
