#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_preprocess/imu_lpf.hpp"

namespace tobas_preprocess
{
class ImuLpfNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<ImuLpf> node_;
};
}  // namespace tobas_preprocess
