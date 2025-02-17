#pragma once

#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/display.hpp>

namespace rviz_common
{
class VisualizationFrame;
class VisualizationManager;
class DisplayGroup;
}  // namespace rviz_common

namespace rviz
{
class RvizFrameManager
{
public:
  explicit RvizFrameManager(const std::string& node_name);

  void initialize(const QString& config_path, QWidget* parent = nullptr);

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rvizNode();
  rclcpp::Node::SharedPtr rawNode();

  QWidget* widget();

  QString getFixedFrame() const;
  void setFixedFrame(const QString& frame);
  void resetTime();

  rviz_common::Display* getDisplay(const QString& name);

private:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> node_;

  rviz_common::VisualizationFrame* frame_;
  rviz_common::VisualizationManager* manager_;
  rviz_common::DisplayGroup* display_group_;

  void removeDefaultColorMaterials();
};
}  // namespace rviz
