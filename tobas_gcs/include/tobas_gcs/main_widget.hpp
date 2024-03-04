#pragma once

#include <ros/ros.h>
#include <QWidget>

namespace tobas_gcs
{
class MainWidget : public QWidget
{
  Q_OBJECT

public:
  explicit MainWidget(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};
}  // namespace tobas_gcs
