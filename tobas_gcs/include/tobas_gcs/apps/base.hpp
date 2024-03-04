#pragma once

#include <string_view>
#include <ros/ros.h>
#include <QtWidgets>

namespace tobas_gcs
{
class BaseAppWidget : public QWidget
{
  Q_OBJECT

public:
  explicit BaseAppWidget(QWidget* parent, ros::NodeHandle& nh, ros::NodeHandle& pnh);

protected:
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;
};
}  // namespace tobas_gcs
