#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_preprocess/battery_lpf.hpp"

namespace tobas_preprocess
{
class BatteryLpfNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<BatteryLpf> node_;
};
}  // namespace tobas_preprocess
