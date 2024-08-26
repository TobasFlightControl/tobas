#pragma once

#include <rviz_common/visualization_frame.hpp>

namespace qt
{
rviz_common::VisualizationFrame* createRvizFrame(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
  const QString& config_path,
  QWidget* parent = nullptr);
}
