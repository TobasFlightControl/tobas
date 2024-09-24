#pragma once

#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>

namespace qt
{
class RvizFrameManager
{
public:
  explicit RvizFrameManager(const std::string& node_name);

  void initialize(const QString& config_path, QWidget* parent = nullptr);

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rvizNode();
  rclcpp::Node::SharedPtr rawNode();
  rviz_common::VisualizationFrame* frame();

private:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> node_;
  rviz_common::VisualizationFrame* frame_;
};
}  // namespace qt
