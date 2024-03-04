#include "../include/tobas_gcs/main_widget.hpp"

namespace tobas_gcs
{
MainWidget::MainWidget(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
{
  setWindowTitle("Tobas");
}
}  // namespace tobas_gcs
