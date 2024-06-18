#include <pluginlib/class_list_macros.hpp>

#include "./matrix_euler_converter_nodelet.hpp"

namespace tobas_preprocess
{
void MatrixEulerConverterNodelet::onInit()
{
  NODELET_INFO("Initializing Matrix Euler Converter Nodelet.");
  node_.reset(new MatrixEulerConverter(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace tobas_preprocess

PLUGINLIB_EXPORT_CLASS(tobas_preprocess::MatrixEulerConverterNodelet, nodelet::Nodelet);
