#include <pluginlib/class_list_macros.hpp>

#include "./matrix_euler_converter_nodelet.hpp"

namespace tobas_preprocess
{
void MatrixEulerConverterNodelet::onInit()
{
  NODELET_INFO("Initializing Matrix Euler Converter Nodelet.");

  const auto& nh = getNodeHandle();
  const auto& pnh = getPrivateNodeHandle();
  const auto& name = getName();

  node_.reset(new MatrixEulerConverter(nh, pnh, name));
}
}  // namespace tobas_preprocess

PLUGINLIB_EXPORT_CLASS(tobas_preprocess::MatrixEulerConverterNodelet, nodelet::Nodelet);
