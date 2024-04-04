#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_preprocess/matrix_euler_converter.hpp"

namespace tobas_preprocess
{
class MatrixEulerConverterNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<MatrixEulerConverter> node_;
};
}  // namespace tobas_preprocess
