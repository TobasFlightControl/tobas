#pragma once

#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

namespace rviz_common
{
class VisualizationFramePy : public VisualizationFrame
{
  using self = VisualizationFramePy;
  using super = VisualizationFrame;

public:
  explicit VisualizationFramePy(QWidget* parent = nullptr);
  ~VisualizationFramePy();

  void initialize(const QString& display_config_file = "");

private:
  rviz_common::ros_integration::RosClientAbstraction client_;
};
}  // namespace rviz_common
