#pragma once

#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/visualization_frame.hpp>

namespace qt
{
rviz_common::VisualizationFrame* createRvizFrame(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node_if,
  const QString& config_path,
  QWidget* parent = nullptr);
}  // namespace qt
