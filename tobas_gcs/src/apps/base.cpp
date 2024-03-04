#include "../../include/tobas_gcs/apps/base.hpp"

namespace tobas_gcs
{
BaseAppWidget::BaseAppWidget(QWidget* parent, ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : QWidget(parent), nh_(nh), pnh_(pnh)
{
}
}  // namespace tobas_gcs
